/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/torque_converter.hpp"

#include "rclcpp/rclcpp.hpp"

TorqueConverter::TorqueConverter(std::shared_ptr<RobotDescription> robot_description)
{
    try {
        m_backpack_node = robot_description->findNode("backpack");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not find backpack node in robot description.");
    }

    try {
        std::vector<std::string> joint_names = m_backpack_node->getRelativeJointNames();
        std::vector<RobotNode::SharedPtr> joint_nodes = robot_description->findNodes(joint_names);
        for (auto& joint_node : joint_nodes) {
            m_joint_nodes.push_back(joint_node);
            m_joint_nodes_map[joint_node->getName()] = joint_node;
            m_jacobian_position_map[joint_node->getName()] = &RobotNode::getGlobalPositionJacobian;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not find joint nodes in robot description.");
    }
}

std::vector<std::string> TorqueConverter::getJointNames() const
{
    return m_backpack_node->getRelativeJointNames();
}

RobotNode::JointNameToValueMap TorqueConverter::getDynamicalJointAccelerations(
    RobotNode::JointNameToValueMap joint_positions, RobotNode::JointNameToValueMap joint_torques) const
{
    try {
        RobotNode::JointNameToValueMap joint_accelerations;
        for (const auto& joint_pair : m_joint_nodes_map) {
            double joint_torque = joint_torques.at(joint_pair.first);
            double joint_acceleration = joint_pair.second->getDynamicalJointAcceleration(joint_torque, joint_positions);
            joint_accelerations[joint_pair.first] = joint_acceleration;
        }
        return joint_accelerations;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not calculate joint acceleration %s.", e.what());
        return RobotNode::JointNameToValueMap();
    }
}

RobotNode::JointNameToValueMap TorqueConverter::getDynamicalTorques(RobotNode::JointNameToValueMap joint_positions,
    RobotNode::JointNameToValueMap joint_velocities, RobotNode::JointNameToValueMap joint_accelerations) const
{
    try {
        RobotNode::JointNameToValueMap joint_torques;
        Eigen::VectorXd joint_torque_values
            = m_backpack_node->getDynamicalTorque(joint_positions, joint_velocities, joint_accelerations);
        for (unsigned long int i = 0; i < m_joint_nodes.size(); i++) {
            joint_torques[m_joint_nodes[i]->getName()] = joint_torque_values(i);
        }
        return joint_torques;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not calculate dynamical torque %s.", e.what());
        return RobotNode::JointNameToValueMap();
    }
}