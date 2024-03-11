/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/torque_converter.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>

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

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("march_description");
    std::string urdf_file = package_share_directory + "/urdf/march8/hennie_with_koen.urdf";

    if (!m_urdf_model.initFile(urdf_file)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialize URDF model from file %s.", urdf_file.c_str());
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(m_urdf_model, kdl_tree)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialize KDL tree from URDF model.");
    }

    if (!kdl_tree.getChain("backpack", "L_ground", m_kdl_chain_left)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialize KDL chain from KDL tree.");
    }

    if (!kdl_tree.getChain("backpack", "R_ground", m_kdl_chain_right)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialize KDL chain from KDL tree.");
    }

    // if (!m_kdl_rne_solver.init(kdl_tree, KDL::Vector(0.0, 0.0, -9.81))) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialize KDL RNE solver.");
    // }

    m_joint_names_left = { "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle" };
    m_joint_names_right = { "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle" };

    m_inertial_to_backpack_orientation = Eigen::Quaterniond::Identity();
    m_gravity_vector = Eigen::Vector3d(0.0, 0.0, -9.81);
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
    // try {
    //     RobotNode::JointNameToValueMap joint_torques;
    //     Eigen::VectorXd joint_torque_values
    //         = m_robot_description->findNode("backpack")->getDynamicalTorque(joint_positions, joint_velocities, joint_accelerations);
    //     for (unsigned long int i = 0; i < m_joint_nodes.size(); i++) {
    //         joint_torques[m_joint_nodes[i]->getName()] = joint_torque_values(i);
    //     }
    //     return joint_torques;
    // } catch (const std::exception& e) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not calculate dynamical torque %s.", e.what());
    //     return RobotNode::JointNameToValueMap();
    // }
    Eigen::VectorXd joint_torque_values = rnea(joint_positions, joint_velocities, joint_accelerations);
    RobotNode::JointNameToValueMap joint_torques;
    for (unsigned long int i = 0; i < m_joint_nodes.size(); i++) {
        joint_torques[m_joint_nodes[i]->getName()] = joint_torque_values(i);
    }
    return joint_torques;
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
    Eigen::MatrixXd jacobian_inverse, jacobian;

    RobotNode::SharedPtr node = m_robot_description->findNode(node_name);
    jacobian.noalias() = m_robot_description->getInertialOrientation() * node->getGlobalPositionJacobian(joint_positions);
    jacobian_inverse.noalias() = jacobian.completeOrthogonalDecomposition().pseudoInverse();

    external_force_vector.noalias() = jacobian_inverse * node->convertAbsoluteJointValuesToVectorXd(external_torques);
    return external_force_vector;
}

void TorqueConverter::setInertialToBackpackOrientation(const Eigen::Quaterniond& orientation)
{
    m_inertial_to_backpack_orientation = orientation;
}

Eigen::VectorXd TorqueConverter::rnea(
    const RobotNode::JointNameToValueMap& joint_positions, 
    const RobotNode::JointNameToValueMap& joint_velocities, 
    const RobotNode::JointNameToValueMap& joint_accelerations) const
{
    Eigen::Vector3d backpack_gravity_vector = m_inertial_to_backpack_orientation * m_gravity_vector;

    KDL::ChainIdSolver_RNE kdl_rne_left_solver(
        m_kdl_chain_left, 
        KDL::Vector(backpack_gravity_vector.x(), backpack_gravity_vector.y(), backpack_gravity_vector.z()));

    KDL::ChainIdSolver_RNE kdl_rne_right_solver(
        m_kdl_chain_right, 
        KDL::Vector(backpack_gravity_vector.x(), backpack_gravity_vector.y(), backpack_gravity_vector.z()));

    KDL::JntArray kdl_joint_positions_left(m_kdl_chain_left.getNrOfJoints());
    KDL::JntArray kdl_joint_velocities_left(m_kdl_chain_left.getNrOfJoints());
    KDL::JntArray kdl_joint_accelerations_left(m_kdl_chain_left.getNrOfJoints());

    KDL::JntArray kdl_joint_positions_right(m_kdl_chain_right.getNrOfJoints());
    KDL::JntArray kdl_joint_velocities_right(m_kdl_chain_right.getNrOfJoints());
    KDL::JntArray kdl_joint_accelerations_right(m_kdl_chain_right.getNrOfJoints());

    kdl_joint_positions_left.data.setZero();
    kdl_joint_velocities_left.data.setZero();
    kdl_joint_accelerations_left.data.setZero();

    kdl_joint_positions_right.data.setZero();
    kdl_joint_velocities_right.data.setZero();
    kdl_joint_accelerations_right.data.setZero();

    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Left KDL joint numbers: %d", m_kdl_chain_left.getNrOfJoints());
    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Right KDL joint numbers: %d", m_kdl_chain_right.getNrOfJoints());

    for (unsigned long int i = 0; i < m_kdl_chain_left.getNrOfJoints(); i++) {
        kdl_joint_positions_left(i) = joint_positions.at(m_joint_names_left[i]);
        kdl_joint_velocities_left(i) = joint_velocities.at(m_joint_names_left[i]);
        kdl_joint_accelerations_left(i) = joint_accelerations.at(m_joint_names_left[i]);
    }

    for (unsigned long int i = 0; i < m_kdl_chain_right.getNrOfJoints(); i++) {
        kdl_joint_positions_right(i) = joint_positions.at(m_joint_names_right[i]);
        kdl_joint_velocities_right(i) = joint_velocities.at(m_joint_names_right[i]);
        kdl_joint_accelerations_right(i) = joint_accelerations.at(m_joint_names_right[i]);
    }

    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Joint arrays initialized");

    // KDL::Wrench left_ground_wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
    // KDL::Wrench right_ground_wrench(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
    

    // KDL::Wrenches wrench_map_left, wrench_map_right;
    // wrench_map_left.push_back(left_ground_wrench);
    // wrench_map_right.push_back(right_ground_wrench);

    KDL::Wrenches wrench_map_left(m_kdl_chain_left.getNrOfSegments());
    KDL::Wrenches wrench_map_right(m_kdl_chain_right.getNrOfSegments());

    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Wrench map initialized");

    KDL::JntArray torques_left(m_kdl_chain_left.getNrOfJoints());
    KDL::JntArray torques_right(m_kdl_chain_right.getNrOfJoints());
    kdl_rne_left_solver.CartToJnt(kdl_joint_positions_left, kdl_joint_velocities_left, kdl_joint_accelerations_left, wrench_map_left, torques_left);
    kdl_rne_right_solver.CartToJnt(kdl_joint_positions_right, kdl_joint_velocities_right, kdl_joint_accelerations_right, wrench_map_right, torques_right);

    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Torques calculated");

    Eigen::VectorXd torques = Eigen::VectorXd::Zero(m_joint_nodes.size());

    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Eigen vector initialized. Size: %d", torques.size());

    // for (unsigned long int i = 0; i < torques_left.rows(); i++) {
    //     RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Torque: %f", torques_left(i));
    // }
    // for (unsigned long int i = 0; i < torques_right.rows(); i++) {
    //     RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Torque: %f", torques_right(i));
    // }

    for (unsigned long int i = 0; i < 4; i++) {
        // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Torque: %f", torques_left(i));
        torques(i) = torques_left(i);
    }
    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Torques converted to Eigen vector");
    for (unsigned long int i = 0; i < 4; i++) {
        // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Torque: %f", torques_right(i));
        torques(i + 4) = torques_right(i);
    }

    // RCLCPP_INFO(rclcpp::get_logger("state_estimator"), "Torques converted to Eigen vector");

    return torques;
}

Eigen::VectorXd TorqueConverter::filterToeJointValues(const Eigen::VectorXd& joint_values) const
{
    Eigen::VectorXd filtered_joint_values(joint_values.size() - 2);
    for (unsigned long int i = 0; i < joint_values.size(); i++) {
        if (m_joint_nodes[i]->getName() != "left_toe" && m_joint_nodes[i]->getName() != "right_toe") {
            filtered_joint_values(i) = joint_values(i);
        }
    }
    return filtered_joint_values;
}

Eigen::VectorXd TorqueConverter::convertToEigenVector(const KDL::JntArray& kdl_jnt_array) const
{
    Eigen::VectorXd eigen_vector(kdl_jnt_array.rows());
    for (unsigned long int i = 0; i < kdl_jnt_array.rows(); i++) {
        eigen_vector(i) = kdl_jnt_array(i);
    }
    return eigen_vector;
}

Eigen::VectorXd TorqueConverter::convertToEigenVector(const RobotNode::JointNameToValueMap& joint_values) const
{
    Eigen::VectorXd eigen_vector(joint_values.size());
    for (unsigned long int i = 0; i < joint_values.size(); i++) {
        eigen_vector(i) = joint_values.at(m_joint_nodes[i]->getName());
    }
    return eigen_vector;
}

KDL::JntArray TorqueConverter::convertToKDLJntArray(const RobotNode::JointNameToValueMap& joint_values) const
{
    KDL::JntArray kdl_joint_values(joint_values.size());
    for (unsigned long int i = 0; i < joint_values.size(); i++) {
        kdl_joint_values(i) = joint_values.at(m_joint_nodes[i]->getName());
    }
    return kdl_joint_values;
}

RobotNode::JointNameToValueMap TorqueConverter::expandJointValuesWithToes(const RobotNode::JointNameToValueMap& joint_values) const
{
    RobotNode::JointNameToValueMap expanded_joint_values = joint_values;
    expanded_joint_values["left_toe"] = 0.0;
    expanded_joint_values["right_toe"] = 0.0;
    return expanded_joint_values;
}