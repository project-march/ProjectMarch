/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/sensor_fusion.hpp"

SensorFusion::SensorFusion(const RobotDescription::SharedPtr robot_description)
{
    m_robot_description = robot_description;
    m_torque_converter = std::make_unique<TorqueConverter>(robot_description);
}

void SensorFusion::configureJointNames(const std::vector<std::string>& joint_names)
{
    m_joint_positions.clear();
    m_joint_velocities.clear();
    m_joint_accelerations.clear();
    m_joint_total_torques.clear();

    for (const auto& joint_name : joint_names) {
        m_joint_positions[joint_name] = 0.0;
        m_joint_velocities[joint_name] = 0.0;
        m_joint_accelerations[joint_name] = 0.0;
        m_joint_total_torques[joint_name] = 0.0;
    }
}

void SensorFusion::updateQuaternionValues(const geometry_msgs::msg::Quaternion* quaternion)
{
    m_quaternion = Eigen::Quaterniond(quaternion->w, quaternion->x, quaternion->y, quaternion->z);
    m_robot_description->setInertialOrientation(m_quaternion);
}

void SensorFusion::updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    for (unsigned int i = 0; i < joint_state->name.size(); i++) {
        m_joint_positions[joint_state->name[i]] = joint_state->position[i];
        m_joint_velocities[joint_state->name[i]] = joint_state->velocity[i];
        m_joint_total_torques[joint_state->name[i]] = joint_state->effort[i];
    }

    m_joint_accelerations
        = m_torque_converter->getDynamicalJointAccelerations(m_joint_positions, m_joint_total_torques);
}

uint8_t SensorFusion::updateStanceLeg(
    const geometry_msgs::msg::Point* left_foot_position, const geometry_msgs::msg::Point* right_foot_position)
{
    uint8_t stance_leg = 0b11;
    const double margin = 0.01;

    if (abs(left_foot_position->x - right_foot_position->x) <= margin) {
        stance_leg = 0b11;
    } else if (left_foot_position->x + margin <= right_foot_position->x) {
        stance_leg = 0b10;
    } else if (left_foot_position->x - margin > right_foot_position->x) {
        stance_leg = 0b01;
    }

    m_robot_description->setStanceLeg(stance_leg,
        Eigen::Vector3d(left_foot_position->x, left_foot_position->y, left_foot_position->z),
        Eigen::Vector3d(right_foot_position->x, right_foot_position->y, right_foot_position->z));
    return stance_leg;
}

Eigen::Vector3d SensorFusion::getCOM() const
{
    Eigen::Vector3d com_body_position = m_robot_description->findNode("com")->getGlobalPosition(m_joint_positions);
    Eigen::Vector3d com_inertial_position;
    com_inertial_position.noalias() = m_quaternion * com_body_position;
    return com_inertial_position;
}

geometry_msgs::msg::Point SensorFusion::getZMP() const
{
    Eigen::Vector3d zmp_position = m_robot_description->findNode("zmp")->getGlobalPosition(m_joint_positions);
    geometry_msgs::msg::Point zmp;
    zmp.x = zmp_position.x();
    zmp.y = zmp_position.y();
    zmp.z = zmp_position.z();
    return zmp;
}

std::vector<geometry_msgs::msg::Pose> SensorFusion::getFootPoses() const
{
    std::vector<geometry_msgs::msg::Pose> foot_poses;
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes({ "L_foot", "R_foot" });

    for (long unsigned int i = 0; i < feet_nodes.size(); i++) {
        geometry_msgs::msg::Pose foot_pose;
        Eigen::Vector3d foot_position = feet_nodes[i]->getGlobalPosition(m_joint_positions);
        foot_pose.position.x = foot_position.x();
        foot_pose.position.y = foot_position.y();
        foot_pose.position.z = foot_position.z();
        foot_pose.orientation.x = 0.0;
        foot_pose.orientation.y = 0.0;
        foot_pose.orientation.z = 0.0;
        foot_pose.orientation.w = 1.0;
        foot_poses.push_back(foot_pose);
    }

    return foot_poses;
}