/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/state_estimator.hpp"
#include <cmath>

StateEstimator::StateEstimator(const RobotDescription::SharedPtr robot_description, const std::string& urdf_file_path)
{
    m_robot_description = robot_description;
    m_torque_converter = std::make_unique<TorqueConverter>(robot_description, urdf_file_path);
    m_state_estimator_timestep = 0.05; // TODO: Make this a parameter
    
    m_current_stance_leg = 0b11;
    m_left_foot_force = Eigen::Vector3d::Zero();
    m_right_foot_force = Eigen::Vector3d::Zero();

    // m_state_posterior = EKFState();

    // // TODO: get from parameter server
    // m_state_posterior.imu_position = Eigen::Vector3d(0.0, 0.0, 0.77);
    // m_state_posterior.left_foot_position = Eigen::Vector3d(0.38, 0.037, 0.0);
    // m_state_posterior.right_foot_position = Eigen::Vector3d(0.38, -0.049, 0.0);
    // m_timestep = 1e-3;

    // m_process_noise_acceleration = Eigen::Vector3d(2.16e-5, 2.16e-5, 2.16e-5); // 1.94e-5);
    // m_process_noise_angular_velocity = Eigen::Vector3d(0.118174, 0.118174, 0.118174);
    // setProcessNoiseCovarianceMatrix(
    //     m_process_noise_acceleration, m_process_noise_acceleration, m_process_noise_angular_velocity,
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23), // feet position
    //     Eigen::Vector3d(3e-7, 3e-7, 3e-7), // accelerometer bias
    //     Eigen::Vector3d(1.454e-5, 1.454e-5, 1.454e-5), // gyroscope bias
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23) // feet slippage
    // );
    // setMeasurementNoiseCovarianceMatrix(
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23), // feet position
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23) // feet slippage
    // );
}

void StateEstimator::configureJointNames(const std::vector<std::string>& joint_names)
{
    m_joint_names = joint_names;

    m_joint_positions.clear();
    m_joint_velocities.clear();
    m_joint_accelerations.clear();
    m_joint_total_torques.clear();
    m_joint_external_torques.clear();

    for (const auto& joint_name : joint_names) {
        m_joint_positions[joint_name] = 0.0;
        m_joint_velocities[joint_name] = 0.0;
        m_joint_accelerations[joint_name] = 0.0;
        m_joint_total_torques[joint_name] = 0.0;
        m_joint_dynamical_torques[joint_name] = 0.0;
        m_joint_external_torques[joint_name] = 0.0;
    }
}

void StateEstimator::configureStanceThresholds(const double& left_foot_threshold, const double& right_foot_threshold)
{
    m_left_foot_threshold = left_foot_threshold;
    m_right_foot_threshold = right_foot_threshold;
}

void StateEstimator::updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    for (unsigned int i = 0; i < joint_state->name.size(); i++) {
        m_joint_accelerations[joint_state->name[i]] = (joint_state->velocity[i] - m_joint_velocities[joint_state->name[i]]) / m_state_estimator_timestep;
        m_joint_positions[joint_state->name[i]] = joint_state->position[i];
        m_joint_velocities[joint_state->name[i]] = joint_state->velocity[i];
        m_joint_total_torques[joint_state->name[i]] = joint_state->effort[i];
    }
}

void StateEstimator::updateImuState(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    m_recent_imu_msg = imu;
    m_quaternion = Eigen::Quaterniond(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    m_robot_description->setInertialOrientation(m_quaternion);
    m_torque_converter->setInertialOrientation(m_quaternion);
}

void StateEstimator::updateDynamicsState()
{
    m_joint_dynamical_torques = m_torque_converter->getDynamicalTorques(m_joint_positions, m_joint_velocities, m_joint_accelerations);
    m_joint_external_torques = m_torque_converter->getExternalTorques(m_joint_total_torques, m_joint_dynamical_torques);

    m_left_foot_force = m_torque_converter->getExternalForceByNode("L_sole", m_joint_positions, m_joint_external_torques);
    m_right_foot_force = m_torque_converter->getExternalForceByNode("R_sole", m_joint_positions, m_joint_external_torques);

    updateCurrentStanceLeg(m_left_foot_force.z(), m_right_foot_force.z(),
        m_robot_description->findNode("L_sole")->getGlobalPosition(m_joint_positions),
        m_robot_description->findNode("R_sole")->getGlobalPosition(m_joint_positions));
}

Eigen::Vector3d StateEstimator::getLeftFootForce() const
{
    return m_left_foot_force;
}

Eigen::Vector3d StateEstimator::getRightFootForce() const
{
    return m_right_foot_force;
}

uint8_t StateEstimator::getCurrentStanceLeg() const
{
    return m_current_stance_leg;
}

uint8_t StateEstimator::getNextStanceLeg(const double& left_foot_position, const double& right_foot_position) const
{
    const double margin = 0.05;

    if (left_foot_position + margin <= right_foot_position) {
        return 0b10;
    } else if (left_foot_position - margin > right_foot_position) {
        return 0b01;
    } else {
        return 0b11;
    }
}

void StateEstimator::updateCurrentStanceLeg(
    const double& left_foot_torque, const double& right_foot_torque,
    const Eigen::Vector3d& left_foot_position, const Eigen::Vector3d& right_foot_position)
{
    m_current_stance_leg = ((right_foot_torque > m_left_foot_threshold) << 1) | (left_foot_torque > m_right_foot_threshold);
    m_robot_description->setStanceLeg(m_current_stance_leg, 
        Eigen::Vector3d(left_foot_position.x(), left_foot_position.y(), left_foot_position.z()),
        Eigen::Vector3d(right_foot_position.x(), right_foot_position.y(), right_foot_position.z()));
}

Eigen::Quaterniond StateEstimator::getInertialOrientation() const
{
    // return m_quaternion;
    return m_quaternion;
}

sensor_msgs::msg::JointState StateEstimator::getEstimatedJointState() const
{
    sensor_msgs::msg::JointState estimated_joint_state;
    estimated_joint_state.name = m_joint_names;

    for (const auto& joint_name : m_joint_names)
    {
        estimated_joint_state.position.push_back(m_joint_positions.at(joint_name));
        estimated_joint_state.velocity.push_back(m_joint_velocities.at(joint_name));
        estimated_joint_state.effort.push_back(m_joint_total_torques.at(joint_name));
    }

    return estimated_joint_state;
}

RobotNode::JointNameToValueMap StateEstimator::getJointPositions() const
{
    return m_joint_positions;
}

Eigen::Vector3d StateEstimator::getCOM() const
{
    Eigen::Vector3d com_body_position = m_robot_description->findNode("com")->getGlobalPosition(m_joint_positions);
    Eigen::Vector3d com_inertial_position;
    com_inertial_position.noalias() = m_quaternion * com_body_position; // + m_state_posterior.imu_position;
    return com_inertial_position;
}

Eigen::Vector3d StateEstimator::getCOMVelocity() const
{
    Eigen::Vector3d com_body_velocity = m_robot_description->findNode("com")->getGlobalVelocity(m_joint_positions, m_joint_velocities);
    Eigen::Vector3d com_inertial_velocity;
    com_inertial_velocity.noalias() = m_quaternion * com_body_velocity; // + m_state_posterior.imu_velocity;
    return com_inertial_velocity;
}

geometry_msgs::msg::Point StateEstimator::getZMP() const
{
    Eigen::Vector3d zmp_position = m_robot_description->findNode("zmp")->getGlobalPosition(m_joint_positions);
    geometry_msgs::msg::Point zmp;
    zmp.x = zmp_position.x();
    zmp.y = zmp_position.y();
    zmp.z = zmp_position.z();
    return zmp;
}

std::vector<geometry_msgs::msg::Pose> StateEstimator::getFootPoses() const
{
    std::vector<geometry_msgs::msg::Pose> foot_poses;
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes({ "L_foot", "R_foot" });

    for (long unsigned int i = 0; i < feet_nodes.size(); i++) {
        geometry_msgs::msg::Pose foot_pose;
        Eigen::Vector3d foot_position = feet_nodes[i]->getGlobalPosition(m_joint_positions);
        Eigen::Quaterniond foot_orientation = Eigen::Quaterniond(feet_nodes[i]->getGlobalRotation(m_joint_positions));
        foot_pose.position.x = foot_position.x();
        foot_pose.position.y = foot_position.y();
        foot_pose.position.z = foot_position.z();
        foot_pose.orientation.x = foot_orientation.x();
        foot_pose.orientation.y = foot_orientation.y();
        foot_pose.orientation.z = foot_orientation.z();
        foot_pose.orientation.w = foot_orientation.w();
        foot_poses.push_back(foot_pose);
    }

    return foot_poses;
}

std::vector<double> StateEstimator::getFootContactHeight() const
{
    std::vector<double> foot_contact_height;
    // std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes({ "L_foot", "R_foot" });

    // for (const auto& foot_node : feet_nodes) {
    //     Eigen::Vector3d foot_position = foot_node->getGlobalPosition(m_joint_positions);
    //     foot_position.noalias() 
    //         = m_quaternion * foot_position + m_state_posterior.imu_position;
    //     foot_contact_height.push_back(foot_position.z());
    // }

    return foot_contact_height;
}

std::vector<double> StateEstimator::getJointAcceleration(const std::vector<std::string>& joint_names) const
{
    std::vector<double> joint_acceleration;
    for (const auto& joint_name : joint_names) {
        joint_acceleration.push_back(m_joint_accelerations.at(joint_name));
    }
    return joint_acceleration;
}

std::vector<double> StateEstimator::getJointDynamicalTorques(const std::vector<std::string>& joint_names) const
{
    std::vector<double> joint_dynamical_torques;
    for (const auto& joint_name : joint_names) {
        joint_dynamical_torques.push_back(m_joint_dynamical_torques.at(joint_name));
    }
    return joint_dynamical_torques;
}

std::vector<double> StateEstimator::getJointExternalTorques(const std::vector<std::string>& joint_names) const
{
    std::vector<double> joint_total_torques;
    for (const auto& joint_name : joint_names) {
        joint_total_torques.push_back(m_joint_external_torques.at(joint_name));
    }
    return joint_total_torques;
}

std::vector<Eigen::Vector3d> StateEstimator::getWorldTorqueInLegs() const
{
    std::vector<Eigen::Vector3d> world_torque_in_legs
        = m_torque_converter->getWorldTorqueInLegs(m_joint_positions, m_joint_total_torques);

    // Orientate the torque vectors to the world frame.
    for (auto& torque : world_torque_in_legs) {
        torque.noalias() = m_quaternion.inverse() * torque;
    }

    return world_torque_in_legs;
}

std::vector<Eigen::Vector3d> StateEstimator::getWorldForceInLegs() const
{
    std::vector<Eigen::Vector3d> world_torque_in_legs = getWorldTorqueInLegs();
    std::vector<Eigen::Vector3d> world_force_in_legs;

    for (const auto& torque : world_torque_in_legs) {
        Eigen::Vector3d force = m_quaternion.inverse() * torque;
        world_force_in_legs.push_back(force);
    }

    return world_force_in_legs;
}