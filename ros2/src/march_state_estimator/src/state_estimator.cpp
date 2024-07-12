/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/state_estimator.hpp"
#include <cmath>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/rnea.hpp"

StateEstimator::StateEstimator(const std::string& urdf_file_path)
{
    configureRobotModel(urdf_file_path);
    configureJointState(m_robot_model.names);
    initializeCurrentStanceLeg();
    initalizeFeetForce();
}

void StateEstimator::updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    for (const auto& joint_name : m_joint_names) {
        auto it = std::find(joint_state->name.begin(), joint_state->name.end(), joint_name);
        if (it == joint_state->name.end()) {
            continue;
        }
        std::size_t joint_id = std::distance(joint_state->name.begin(), it);
        m_joint_acceleration[joint_id] = (joint_state->velocity[joint_id] - m_joint_velocity[joint_id]) / m_timestep;
        m_joint_position[joint_id] = joint_state->position[joint_id];
        m_joint_velocity[joint_id] = joint_state->velocity[joint_id];
        m_joint_total_torque[joint_id] = joint_state->effort[joint_id];
    }
    pinocchio::forwardKinematics(m_robot_model, *m_robot_data, m_joint_position, m_joint_velocity, m_joint_acceleration);
}

void StateEstimator::updateImuState(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    m_quaternion = Eigen::Quaterniond(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
}

void StateEstimator::updateDynamicsState()
{
    m_joint_dynamical_torque = pinocchio::rnea(m_robot_model, *m_robot_data, m_joint_position, m_joint_velocity, m_joint_acceleration);
    m_joint_external_torque = m_joint_total_torque - m_joint_dynamical_torque;

    m_left_foot_force = computeTotalForce("left_ankle_dpf");
    m_right_foot_force = computeTotalForce("right_ankle_dpf");

    updateCurrentStanceLeg(m_left_foot_force.z(), m_right_foot_force.z(),
        m_robot_data->oMi[m_robot_model.getJointId("left_ankle_ie")].translation(),
        m_robot_data->oMi[m_robot_model.getJointId("right_ankle_ie")].translation());
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
}

Eigen::Quaterniond StateEstimator::getInertialOrientation() const
{
    return m_quaternion;
}

sensor_msgs::msg::JointState StateEstimator::getEstimatedJointState() const
{
    sensor_msgs::msg::JointState estimated_joint_state;

    // Alphabetize joint names for consistency
    estimated_joint_state.name.resize(m_joint_names.size());
    std::copy(m_joint_names.begin(), m_joint_names.end(), estimated_joint_state.name.begin());
    std::sort(estimated_joint_state.name.begin(), estimated_joint_state.name.end());

    for (const auto& joint_name : estimated_joint_state.name) {
        auto it = std::find(m_joint_names.begin(), m_joint_names.end(), joint_name);
        if (it == m_joint_names.end()) {
            continue;
        }
        std::size_t joint_id = std::distance(m_joint_names.begin(), it);
        estimated_joint_state.position.push_back(m_joint_position[joint_id]);
        estimated_joint_state.velocity.push_back(m_joint_velocity[joint_id]);
        estimated_joint_state.effort.push_back(m_joint_total_torque[joint_id]);
    }

    return estimated_joint_state;
}

std::unordered_map<std::string, double> StateEstimator::getJointPosition() const
{
    std::unordered_map<std::string, double> joint_position;
    for (unsigned long int i = 0; i < m_joint_names.size(); i++) {
        joint_position[m_joint_names[i]] = m_joint_position(i);
    }
    return joint_position;
}

Eigen::Vector3d StateEstimator::getCOM() const
{
    // return pinocchio::centerOfMass(m_robot_model, *m_robot_data, m_joint_positions);
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d StateEstimator::getCOMVelocity() const
{
    // return pinocchio::centerOfMass(m_robot_model, *m_robot_data, m_joint_positions, m_joint_velocities);
    return Eigen::Vector3d::Zero();
}

geometry_msgs::msg::Point StateEstimator::getZMP() const
{
    // TODO: Implement ZMP calculation
}

std::vector<geometry_msgs::msg::Pose> StateEstimator::getFootPoses() const
{
    std::vector<geometry_msgs::msg::Pose> foot_poses;
    std::vector<long unsigned int> joint_indices = {
        m_robot_model.getJointId("left_ankle_ie"), 
        m_robot_model.getJointId("right_ankle_ie")};
    
    for (const auto& joint_index : joint_indices) {
        Eigen::Vector3d foot_position = m_robot_data->oMi[joint_index].translation();
        Eigen::Quaterniond foot_orientation(m_robot_data->oMi[joint_index].rotation());
        
        geometry_msgs::msg::Pose foot_pose;
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
    return foot_contact_height;
}

std::vector<double> StateEstimator::getJointAcceleration(const std::vector<std::string>& joint_names) const
{
    std::vector<double> joint_acceleration;

    // Alphabetize joint names for consistency
    std::vector<std::string> joint_names_alphabetized;
    std::copy(m_joint_names.begin(), m_joint_names.end(), joint_names_alphabetized.begin());
    std::sort(joint_names_alphabetized.begin(), joint_names_alphabetized.end());

    for (const auto& joint_name : joint_names_alphabetized) {
        auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
        if (it == joint_names.end()) {
            continue;
        }
        std::size_t joint_id = std::distance(joint_names.begin(), it);
        joint_acceleration.push_back(m_joint_acceleration[joint_id]);
    }

    return joint_acceleration;
}

std::vector<double> StateEstimator::getJointDynamicalTorques(const std::vector<std::string>& joint_names) const
{
    std::vector<double> joint_dynamical_torque;

    // Alphabetize joint names for consistency
    std::vector<std::string> joint_names_alphabetized;
    std::copy(m_joint_names.begin(), m_joint_names.end(), joint_names_alphabetized.begin());
    std::sort(joint_names_alphabetized.begin(), joint_names_alphabetized.end());

    for (const auto& joint_name : joint_names_alphabetized) {
        auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
        if (it == joint_names.end()) {
            continue;
        }
        std::size_t joint_id = std::distance(joint_names.begin(), it);
        joint_dynamical_torque.push_back(m_joint_dynamical_torque[joint_id]);
    }

    return joint_dynamical_torque;
}

std::vector<double> StateEstimator::getJointExternalTorques(const std::vector<std::string>& joint_names) const
{
    std::vector<double> joint_external_torque;

    // Alphabetize joint names for consistency
    std::vector<std::string> joint_names_alphabetized;
    std::copy(m_joint_names.begin(), m_joint_names.end(), joint_names_alphabetized.begin());
    std::sort(joint_names_alphabetized.begin(), joint_names_alphabetized.end());

    for (const auto& joint_name : joint_names_alphabetized) {
        auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
        if (it == joint_names.end()) {
            continue;
        }
        std::size_t joint_id = std::distance(joint_names.begin(), it);
        joint_external_torque.push_back(m_joint_external_torque[joint_id]);
    }

    return joint_external_torque;
}

Eigen::Vector3d StateEstimator::computeTotalForce(const std::string& joint_name) const
{
    Eigen::Vector3d total_force;

    pinocchio::Data::Matrix6x jacobian(6, m_robot_model.nv);
    jacobian.setZero();
    pinocchio::computeJointJacobian(m_robot_model, *m_robot_data, m_joint_position, m_robot_model.getJointId(joint_name), jacobian);

    Eigen::MatrixXd jacobian_transpose = jacobian.topRows(3).transpose();
    total_force.noalias() = jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse() * m_joint_total_torque;

    return total_force;
}

Eigen::Vector3d StateEstimator::computeExternalForce(const std::string& joint_name) const
{
    Eigen::Vector3d external_force;

    pinocchio::Data::Matrix6x jacobian(6, m_robot_model.nv);
    jacobian.setZero();
    pinocchio::computeJointJacobian(m_robot_model, *m_robot_data, m_joint_position, m_robot_model.getJointId(joint_name), jacobian);

    Eigen::MatrixXd jacobian_transpose = jacobian.topRows(3).transpose();
    external_force.noalias() = jacobian_transpose.completeOrthogonalDecomposition().pseudoInverse() * m_joint_external_torque;

    return external_force;
}


// Configuration functions

void StateEstimator::configureRobotModel(const std::string& urdf_file_path)
{
    pinocchio::urdf::buildModel(urdf_file_path, m_robot_model);
    m_robot_data = std::make_unique<pinocchio::Data>(m_robot_model);
}

void StateEstimator::configureJointState(const std::vector<std::string>& joint_names)
{
    m_joint_names = joint_names;
    m_joint_names.erase(m_joint_names.begin()); // Remove root joint

    m_joint_position = Eigen::VectorXd::Zero(m_robot_model.nv);
    m_joint_velocity = Eigen::VectorXd::Zero(m_robot_model.nv);
    m_joint_acceleration = Eigen::VectorXd::Zero(m_robot_model.nv);

    m_joint_total_torque = Eigen::VectorXd::Zero(m_robot_model.nv);
    m_joint_dynamical_torque = Eigen::VectorXd::Zero(m_robot_model.nv);
    m_joint_external_torque = Eigen::VectorXd::Zero(m_robot_model.nv);
}

void StateEstimator::initializeCurrentStanceLeg()
{
    // 0b11 = both legs, 0b01 = left leg, 0b10 = right leg
    m_current_stance_leg = 0b11;
}

void StateEstimator::initalizeFeetForce()
{
    m_left_foot_force = Eigen::Vector3d::Zero();
    m_right_foot_force = Eigen::Vector3d::Zero();
}

void StateEstimator::configureStanceThresholds(const double& left_foot_threshold, const double& right_foot_threshold)
{
    m_left_foot_threshold = left_foot_threshold;
    m_right_foot_threshold = right_foot_threshold;
}