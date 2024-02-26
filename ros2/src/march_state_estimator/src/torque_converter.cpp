/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/torque_converter.hpp"
#include "rclcpp/rclcpp.hpp"

TorqueConverter::TorqueConverter(std::shared_ptr<RobotDescription> robot_description)
{
    m_robot_description = robot_description;
    try {
        RobotNode::SharedPtr root_node = robot_description->findNode("backpack");
        std::vector<std::string> joint_names = root_node->getRelativeJointNames();
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
    return m_robot_description->findNode("backpack")->getRelativeJointNames();
}

RobotNode::JointNameToValueMap TorqueConverter::getDynamicalJointAccelerations(
    const RobotNode::JointNameToValueMap& joint_positions, 
    const RobotNode::JointNameToValueMap& joint_torques) const
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

RobotNode::JointNameToValueMap TorqueConverter::getDynamicalTorques(
    const RobotNode::JointNameToValueMap& joint_positions,
    const RobotNode::JointNameToValueMap& joint_velocities, 
    const RobotNode::JointNameToValueMap& joint_accelerations) const
{
    try {
        RobotNode::JointNameToValueMap joint_torques;
        Eigen::VectorXd joint_torque_values
            = m_robot_description->findNode("backpack")->getDynamicalTorque(joint_positions, joint_velocities, joint_accelerations);
        for (unsigned long int i = 0; i < m_joint_nodes.size(); i++) {
            joint_torques[m_joint_nodes[i]->getName()] = joint_torque_values(i);
        }
        return joint_torques;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not calculate dynamical torque %s.", e.what());
        return RobotNode::JointNameToValueMap();
    }
}

RobotNode::JointNameToValueMap TorqueConverter::getExternalTorques(
    const RobotNode::JointNameToValueMap& joint_total_torques, 
    const RobotNode::JointNameToValueMap& joint_dynamical_torques) const
{
    RobotNode::JointNameToValueMap external_torques;
    for (const auto& joint_total_pair : joint_total_torques) {
        external_torques[joint_total_pair.first] = joint_total_pair.second - joint_dynamical_torques.at(joint_total_pair.first);
    }
    return external_torques;
}

Eigen::Vector3d TorqueConverter::getExternalForceByNode(const std::string& node_name, 
    const RobotNode::JointNameToValueMap& joint_positions, 
    const RobotNode::JointNameToValueMap& external_torques) const
{
    Eigen::VectorXd external_force_vector;
    Eigen::MatrixXd jacobian_inverse;

    RobotNode::SharedPtr node = m_robot_description->findNode(node_name);
    jacobian_inverse.noalias() = (node->getGlobalPositionJacobian(joint_positions).transpose()).completeOrthogonalDecomposition().pseudoInverse();

    external_force_vector.noalias() = jacobian_inverse * node->convertAbsoluteJointValuesToVectorXd(external_torques);
    return external_force_vector;
}

Eigen::VectorXd TorqueConverter::convertToEigenVector(const RobotNode::JointNameToValueMap& joint_values) const
{
    Eigen::VectorXd eigen_vector(joint_values.size());
    for (unsigned long int i = 0; i < joint_values.size(); i++) {
        eigen_vector(i) = joint_values.at(m_joint_nodes[i]->getName());
    }
    return eigen_vector;
}