/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/torque_converter.hpp"

#include "rclcpp/rclcpp.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "urdf/model.h"

#include "march_state_estimator/robot_joint.hpp"
#include <math.h>

TorqueConverter::TorqueConverter(std::shared_ptr<RobotDescription> robot_description, const std::string& urdf_file_path)
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

    // Load the robot description from the URDF file, and parse its legs into KDL chains.
    urdf::Model urdf_model;
    if (!urdf_model.initFile(urdf_file_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"), "Could not load URDF file. Please check the path or the file.");
        return;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"), "Could not parse KDL tree from URDF model.");
        return;
    }

    // Get the left and right legs from the KDL tree and store them in the class.
    if (!kdl_tree.getChain("backpack", "L_ground", m_kdl_chain_leg_left)) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"), "Could not get left leg chain from KDL tree.");
        return;
    }

    if (!kdl_tree.getChain("backpack", "R_ground", m_kdl_chain_leg_right)) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"), "Could not get right leg chain from KDL tree.");
        return;
    }

    // Define the orientation of the robot, and the gravity vector with respect to the world frame.
    m_world_to_backpack_orientation = Eigen::Quaterniond::Identity();

    RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "TorqueConverter initialized.");
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
    // TODO: Change recursive Newton-Euler for one leg at a time.
    return recursiveNewtonEuler(joint_positions, joint_velocities, joint_accelerations);
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

RobotNode::JointNameToValueMap TorqueConverter::getTotalTorques(
    const RobotNode::JointNameToValueMap& joint_external_torques, 
    const RobotNode::JointNameToValueMap& joint_dynamical_torques) const
{
    RobotNode::JointNameToValueMap total_torques;
    for (const auto& joint_external_pair : joint_external_torques) {
        total_torques[joint_external_pair.first] = joint_external_pair.second + joint_dynamical_torques.at(joint_external_pair.first);
    }
    return total_torques;
}

Eigen::Vector3d TorqueConverter::getExternalForceByNode(const std::string& node_name, 
    const RobotNode::JointNameToValueMap& joint_positions, 
    const RobotNode::JointNameToValueMap& external_torques) const
{
    RobotNode::SharedPtr node = m_robot_description->findNode(node_name);
    std::vector<double> external_torques_vector;
    for (const auto& joint_name : node->getJointNames()) {
        external_torques_vector.push_back(external_torques.at(joint_name));
    }

    Eigen::VectorXd external_force_vector;
    Eigen::MatrixXd jacobian_inverse;
    jacobian_inverse.noalias() = (node->getGlobalPositionJacobian(joint_positions).transpose()).completeOrthogonalDecomposition().pseudoInverse();
    external_force_vector.noalias() = jacobian_inverse * Eigen::Map<Eigen::VectorXd>(external_torques_vector.data(), external_torques_vector.size());

    return external_force_vector;
}

std::vector<Eigen::Vector3d> TorqueConverter::getWorldTorqueInLegs(
    const RobotNode::JointNameToValueMap& joint_positions,
    const RobotNode::JointNameToValueMap& joint_torques) const
{
    std::vector<Eigen::Vector3d> world_torque_vectors;
    std::unordered_map<std::string, Eigen::Vector3d> joint_torque_vectors = vectorizeJointTorqueVectors(joint_torques);
    // std::unordered_map<std::string, Eigen::Vector3d> global_torque_vectors = orientateTorqueVectorsToWorld(joint_positions, joint_torque_vectors);
    std::unordered_map<std::string, Eigen::Vector3d> global_torque_vectors = orientateTorqueVectorsToFoot(joint_positions, joint_torque_vectors);

    // Summing the torque in the left leg.
    Eigen::Vector3d left_leg_torque_vector = Eigen::Vector3d::Zero();
    unsigned long int half_joint_idx = floor(m_joint_nodes.size() / 2);
    for (unsigned long int i = 0; i < half_joint_idx; i++) {
        left_leg_torque_vector += global_torque_vectors.at(m_joint_nodes[i]->getName());
    }
    world_torque_vectors.push_back(left_leg_torque_vector);

    // Summing the torque in the right leg.
    Eigen::Vector3d right_leg_torque_vector = Eigen::Vector3d::Zero();
    for (unsigned long int i = half_joint_idx; i < m_joint_nodes.size(); i++) {
        right_leg_torque_vector += global_torque_vectors.at(m_joint_nodes[i]->getName());
    }
    world_torque_vectors.push_back(right_leg_torque_vector);

    return world_torque_vectors;
}

Eigen::VectorXd TorqueConverter::convertJointNameToValueMapToEigenVector(const RobotNode::JointNameToValueMap& joint_values) const
{
    Eigen::VectorXd eigen_vector(joint_values.size());
    for (unsigned long int i = 0; i < joint_values.size(); i++) {
        eigen_vector(i) = joint_values.at(m_joint_nodes[i]->getName());
    }
    return eigen_vector;
}

std::unordered_map<std::string, Eigen::Vector3d> TorqueConverter::vectorizeJointTorqueVectors(const RobotNode::JointNameToValueMap& joint_torques) const
{
    std::unordered_map<std::string, Eigen::Vector3d> joint_torque_vectors;
    std::vector<RobotJoint::SharedPtr> joint_nodes = m_robot_description->getJointNodes();
    for (const auto& joint_node : joint_nodes) {
        Eigen::Vector3d joint_torque_vector = Eigen::Vector3d::Zero();
        double joint_torque = joint_torques.at(joint_node->getName());
        std::vector<double> joint_axis = joint_node->getJointAxis();
        for (unsigned long int i = 0; i < joint_axis.size(); i++) {
            joint_torque_vector(i) = joint_torque * joint_axis[i];
        }
        joint_torque_vectors[joint_node->getName()] = joint_torque_vector;
    }
    return joint_torque_vectors;
}

std::unordered_map<std::string, Eigen::Vector3d> TorqueConverter::orientateTorqueVectorsToWorld(
    const RobotNode::JointNameToValueMap& joint_positions,
    const std::unordered_map<std::string, Eigen::Vector3d>& joint_torque_vectors) const
{
    std::unordered_map<std::string, Eigen::Vector3d> global_torque_vectors;
    std::vector<RobotJoint::SharedPtr> joint_nodes = m_robot_description->getJointNodes();
    for (const auto& joint_node : joint_nodes) {
        Eigen::Matrix3d rotation_matrix = joint_node->getGlobalRotation(joint_positions).transpose();
        Eigen::Vector3d global_torque_vector = rotation_matrix * joint_torque_vectors.at(joint_node->getName());
        global_torque_vectors[joint_node->getName()] = global_torque_vector;
    }
    return global_torque_vectors;
}

std::unordered_map<std::string, Eigen::Vector3d> TorqueConverter::orientateTorqueVectorsToFoot(
    const RobotNode::JointNameToValueMap& joint_positions,
    const std::unordered_map<std::string, Eigen::Vector3d>& joint_torque_vectors) const
{
    std::unordered_map<std::string, Eigen::Vector3d> foot_torque_vectors;
    std::vector<RobotJoint::SharedPtr> joint_nodes = m_robot_description->getJointNodes();

    // Get the rotation matrix of the foot.
    Eigen::Matrix3d left_foot_rotation = m_robot_description->findNode("L_foot")->getGlobalRotation(joint_positions).transpose();
    Eigen::Matrix3d right_foot_rotation = m_robot_description->findNode("R_foot")->getGlobalRotation(joint_positions).transpose();

    // Rotate the torque vectors to the foot frame.
    for (const auto& joint_node : joint_nodes) {
        Eigen::Vector3d foot_torque_vector = Eigen::Vector3d::Zero();
        if (joint_node->getName().find("left") != std::string::npos) {
            foot_torque_vector = left_foot_rotation * (m_robot_description->findNode(joint_node->getName())->getGlobalRotation(joint_positions).transpose() * joint_torque_vectors.at(joint_node->getName()));
        } else if (joint_node->getName().find("right") != std::string::npos) {
            foot_torque_vector = right_foot_rotation * (m_robot_description->findNode(joint_node->getName())->getGlobalRotation(joint_positions).transpose() * joint_torque_vectors.at(joint_node->getName()));
        }
        foot_torque_vectors[joint_node->getName()] = foot_torque_vector;
    }

    return foot_torque_vectors;
}

std::vector<Eigen::Vector3d> TorqueConverter::convertTorqueToForce(
    const RobotNode::JointNameToValueMap& joint_positions,
    const RobotNode::JointNameToValueMap& joint_torques) const
{
    std::vector<Eigen::Vector3d> force_vectors;
    const std::vector<std::string> left_joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle"};
    const std::vector<std::string> right_joint_names = {"right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};

    const unsigned long int leg_joint_size = floor(joint_torques.size() / 2);
    Eigen::VectorXd left_joint_torques = Eigen::VectorXd::Zero(leg_joint_size);
    Eigen::VectorXd right_joint_torques = Eigen::VectorXd::Zero(leg_joint_size);

    for (const auto& joint_node : m_robot_description->getJointNodes()) {
        if (std::find(left_joint_names.begin(), left_joint_names.end(), joint_node->getName()) != left_joint_names.end()) {
            left_joint_torques(std::distance(left_joint_names.begin(), std::find(left_joint_names.begin(), left_joint_names.end(), joint_node->getName()))) = joint_torques.at(joint_node->getName());
        } else if (std::find(right_joint_names.begin(), right_joint_names.end(), joint_node->getName()) != right_joint_names.end()) {
            right_joint_torques(std::distance(right_joint_names.begin(), std::find(right_joint_names.begin(), right_joint_names.end(), joint_node->getName()))) = joint_torques.at(joint_node->getName());
        }
    }

    Eigen::VectorXd left_force_vector = m_robot_description->findNode("L_foot")->getGlobalPositionJacobian(joint_positions).transpose().completeOrthogonalDecomposition().pseudoInverse() * left_joint_torques;
    Eigen::VectorXd right_force_vector = m_robot_description->findNode("R_foot")->getGlobalPositionJacobian(joint_positions).transpose().completeOrthogonalDecomposition().pseudoInverse() * right_joint_torques;

    // Convert the force vectors to Eigen::Vector3d.
    Eigen::Vector3d left_force = Eigen::Map<Eigen::Vector3d>(left_force_vector.data());
    Eigen::Vector3d right_force = Eigen::Map<Eigen::Vector3d>(right_force_vector.data());

    force_vectors.push_back(left_force);
    force_vectors.push_back(right_force);

    return force_vectors;
}

RobotNode::JointNameToValueMap TorqueConverter::recursiveNewtonEuler(
    const RobotNode::JointNameToValueMap& joint_positions,
    const RobotNode::JointNameToValueMap& joint_velocities,
    const RobotNode::JointNameToValueMap& joint_accelerations) const
{
    Eigen::Vector3d backpack_gravity = m_world_to_backpack_orientation * m_world_gravity;

    // Create the KDL chain and solver for the left and right legs with the gravity vector, respectively.
    KDL::ChainIdSolver_RNE solver_left_leg(m_kdl_chain_leg_left, KDL::Vector(backpack_gravity.x(), backpack_gravity.y(), backpack_gravity.z()));
    KDL::ChainIdSolver_RNE solver_right_leg(m_kdl_chain_leg_right, KDL::Vector(backpack_gravity.x(), backpack_gravity.y(), backpack_gravity.z()));

    // Define joint arrays for the joint positions, velocities, and accelerations of the left and right legs.
    KDL::JntArray joint_positions_left_leg(m_kdl_chain_leg_left.getNrOfJoints());
    KDL::JntArray joint_velocities_left_leg(m_kdl_chain_leg_left.getNrOfJoints());
    KDL::JntArray joint_accelerations_left_leg(m_kdl_chain_leg_left.getNrOfJoints());
    joint_positions_left_leg.data.setZero();
    joint_velocities_left_leg.data.setZero();
    joint_accelerations_left_leg.data.setZero();

    KDL::JntArray joint_positions_right_leg(m_kdl_chain_leg_right.getNrOfJoints());
    KDL::JntArray joint_velocities_right_leg(m_kdl_chain_leg_right.getNrOfJoints());
    KDL::JntArray joint_accelerations_right_leg(m_kdl_chain_leg_right.getNrOfJoints());
    joint_positions_right_leg.data.setZero();
    joint_velocities_right_leg.data.setZero();
    joint_accelerations_right_leg.data.setZero();

    // Fill the joint arrays with the joint values.
    unsigned int left_joint_idx = 0;
    unsigned int right_joint_idx = 0;
    for (const auto& joint_node : m_joint_nodes) {
        if (joint_node->getName().find("left") != std::string::npos) {
            joint_positions_left_leg(left_joint_idx) = joint_positions.at(joint_node->getName());
            joint_velocities_left_leg(left_joint_idx) = joint_velocities.at(joint_node->getName());
            joint_accelerations_left_leg(left_joint_idx) = joint_accelerations.at(joint_node->getName());
            left_joint_idx++;
        } else if (joint_node->getName().find("right") != std::string::npos) {
            joint_positions_right_leg(right_joint_idx) = joint_positions.at(joint_node->getName());
            joint_velocities_right_leg(right_joint_idx) = joint_velocities.at(joint_node->getName());
            joint_accelerations_right_leg(right_joint_idx) = joint_accelerations.at(joint_node->getName());
            right_joint_idx++;
        }
    }

    // Get zero wrenches for the left and right legs.
    KDL::Wrenches wrenches_left_leg(m_kdl_chain_leg_left.getNrOfSegments());
    KDL::Wrenches wrenches_right_leg(m_kdl_chain_leg_right.getNrOfSegments());
    for (unsigned long int i = 0; i < m_kdl_chain_leg_left.getNrOfSegments(); i++) {
        wrenches_left_leg[i] = KDL::Wrench::Zero();
    }
    for (unsigned long int i = 0; i < m_kdl_chain_leg_right.getNrOfSegments(); i++) {
        wrenches_right_leg[i] = KDL::Wrench::Zero();
    }

    // Calculate the torques of the left and right legs.
    KDL::JntArray torques_left_leg(m_kdl_chain_leg_left.getNrOfJoints());
    KDL::JntArray torques_right_leg(m_kdl_chain_leg_right.getNrOfJoints());
    torques_left_leg.data.setZero();
    torques_right_leg.data.setZero();

    solver_left_leg.CartToJnt(joint_positions_left_leg, joint_velocities_left_leg, joint_accelerations_left_leg, wrenches_left_leg, torques_left_leg);
    solver_right_leg.CartToJnt(joint_positions_right_leg, joint_velocities_right_leg, joint_accelerations_right_leg, wrenches_right_leg, torques_right_leg);

    // Concatenate the torques of the left and right legs into a single Eigen::VectorXd.
    RobotNode::JointNameToValueMap dynamical_torques;
    for (long int i = 0; i < torques_left_leg.data.rows(); i++) {
        std::string left_joint_name = m_joint_nodes[i]->getName();
        dynamical_torques[left_joint_name] = torques_left_leg(i);
    }
    for (long int i = 0; i < torques_right_leg.data.rows(); i++) {
        std::string right_joint_name = m_joint_nodes[i + torques_left_leg.data.rows()]->getName();
        dynamical_torques[right_joint_name] = torques_right_leg(i);
    }

    return dynamical_torques;
}