/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_node.hpp"

#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <sstream>

void RobotNode::setName(const std::string& name)
{
    m_name = name;
}

void RobotNode::setId(const uint64_t& id)
{
    m_id = id;
}

void RobotNode::setParent(RobotNode::SharedPtr parent)
{
    m_parent = parent;
}

void RobotNode::addChild(RobotNode::SharedPtr child)
{
    m_children.push_back(child);
    child->setParent(shared_from_this());
}

void RobotNode::setJointNodes(std::vector<SharedPtr> absolute_joint_nodes, std::vector<SharedPtr> relative_joint_nodes)
{
    // Set joint nodes in absolute reference frame.
    for (const auto& absolute_joint_node : absolute_joint_nodes) {
        m_joint_symbols_list.append(absolute_joint_node->getJointPosition());
        m_joint_symbols_list.append(absolute_joint_node->getJointVelocity());
        m_joint_symbols_list.append(absolute_joint_node->getJointAcceleration());
    }
    m_joint_nodes = absolute_joint_nodes;

    // Set joint nodes in relative reference frame.
    for (const auto& relative_joint_node : relative_joint_nodes) {
        m_relative_joint_nodes.push_back(relative_joint_node);
        m_relative_joint_symbols_list.append(relative_joint_node->getJointPosition());
        m_relative_joint_symbols_list.append(relative_joint_node->getJointVelocity());
        m_relative_joint_symbols_list.append(relative_joint_node->getJointAcceleration());
    }
}

void RobotNode::setExpressionRelativeInertia(const std::string& expression)
{
    m_relative_inertia_expression = GiNaC::ex(expression, m_relative_joint_symbols_list);
}

void RobotNode::setExpressionGlobalPosition(const std::vector<std::string>& expressions)
{
    setExpression(expressions, m_global_position_expressions);
}

void RobotNode::setExpressionGlobalVelocity(const std::vector<std::string>& expressions)
{
    setExpression(expressions, m_global_velocity_expressions);
}

void RobotNode::setExpressionGlobalAcceleration(const std::vector<std::string>& expressions)
{
    setExpression(expressions, m_global_acceleration_expressions);
}

void RobotNode::setExpressionGlobalRotation(const std::vector<std::string>& expressions)
{
    setExpression(expressions, m_global_rotation_expressions);
}

void RobotNode::setExpressionGlobalPositionJacobian(const std::vector<std::string>& expressions)
{
    setExpression(expressions, m_global_position_jacobian_expressions);
}

void RobotNode::setExpressionGlobalRotationJacobian(const std::vector<std::string>& expressions)
{
    setExpression(expressions, m_global_rotation_jacobian_expressions);
}

void RobotNode::setExpressionDynamicalTorque(const std::vector<std::string>& expressions)
{
    if (m_dynamical_torque_expressions.size() > 0) {
        m_dynamical_torque_expressions.clear();
    }
    for (unsigned long int i = 0; i < expressions.size(); i++) {
        GiNaC::ex expression(expressions[i], m_relative_joint_symbols_list);
        m_dynamical_torque_expressions.push_back(expression);
    }
}

std::string RobotNode::getName() const
{
    return m_name;
}

char RobotNode::getType() const
{
    return m_type;
}

RobotNode::JointSymbol RobotNode::getJointPosition() const
{
    return m_joint_symbol_position;
}

RobotNode::JointSymbol RobotNode::getJointVelocity() const
{
    return m_joint_symbol_velocity;
}

RobotNode::JointSymbol RobotNode::getJointAcceleration() const
{
    return m_joint_symbol_acceleration;
}

RobotNode::SharedPtr RobotNode::getParent() const
{
    return m_parent;
}

std::vector<std::weak_ptr<RobotNode>> RobotNode::getChildren() const
{
    return m_children;
}

std::vector<std::string> RobotNode::getJointNames() const
{
    std::vector<std::string> joint_names;
    for (const auto& joint_node : m_joint_nodes) {
        joint_names.push_back(joint_node->getName());
    }
    return joint_names;
}

std::vector<std::string> RobotNode::getRelativeJointNames() const
{
    std::vector<std::string> joint_names;
    for (const auto& joint_node : m_relative_joint_nodes) {
        if (joint_node.expired()) {
            continue;
        }
        joint_names.push_back(joint_node.lock()->getName());
    }
    return joint_names;
}

RobotNode::JointNameToValueMap RobotNode::getAbsoluteJointValues(const JointNameToValueMap& joint_values) const
{
    JointNameToValueMap absolute_joint_values;
    for (const auto& joint_node : m_joint_nodes) {
        absolute_joint_values[joint_node->getName()] = joint_values.at(joint_node->getName());
    }
    return absolute_joint_values;
}

Eigen::VectorXd RobotNode::convertAbsoluteJointValuesToVectorXd(const JointNameToValueMap& joint_values) const
{
    Eigen::VectorXd joint_values_vector = Eigen::VectorXd::Zero(m_joint_nodes.size());
    for (unsigned long int i = 0; i < m_joint_nodes.size(); i++) {
        joint_values_vector(i) = joint_values.at(m_joint_nodes[i]->getName());
    }
    return joint_values_vector;
}

Eigen::Vector3d RobotNode::getGlobalPosition(const JointNameToValueMap& joint_positions) const
{
    return Eigen::Map<Eigen::Vector3d>(
        evaluateExpression(m_global_position_expressions, m_joint_nodes, WORKSPACE_DIM, 1, joint_positions).data());
}

Eigen::Vector3d RobotNode::getGlobalVelocity(const JointNameToValueMap& joint_positions, 
    const JointNameToValueMap& joint_velocities) const
{
    return Eigen::Map<Eigen::Vector3d>(
        evaluateExpression(m_global_velocity_expressions, m_joint_nodes, WORKSPACE_DIM, 1, joint_positions, joint_velocities).data());
}

Eigen::Vector3d RobotNode::getGlobalAcceleration(const JointNameToValueMap& joint_positions, 
    const JointNameToValueMap& joint_velocities, const JointNameToValueMap& joint_accelerations) const
{
    return Eigen::Map<Eigen::Vector3d>(
        evaluateExpression(m_global_acceleration_expressions, m_joint_nodes, WORKSPACE_DIM, 1, joint_positions, joint_velocities, joint_accelerations).data());
}

Eigen::Matrix3d RobotNode::getGlobalRotation(const JointNameToValueMap& joint_positions) const
{
    return Eigen::Map<Eigen::Matrix3d>(
        evaluateExpression(m_global_rotation_expressions, m_joint_nodes, WORKSPACE_DIM, WORKSPACE_DIM, joint_positions)
            .data());
}

Eigen::MatrixXd RobotNode::getGlobalPositionJacobian(const JointNameToValueMap& joint_positions) const
{
    return evaluateExpression(
        m_global_position_jacobian_expressions, m_joint_nodes, WORKSPACE_DIM, m_joint_nodes.size(), joint_positions);
}

Eigen::MatrixXd RobotNode::getGlobalRotationJacobian(const JointNameToValueMap& joint_positions) const
{
    return evaluateExpression(
        m_global_rotation_jacobian_expressions, m_joint_nodes, WORKSPACE_DIM, m_joint_nodes.size(), joint_positions);
}

Eigen::VectorXd RobotNode::getDynamicalTorque(const JointNameToValueMap& joint_positions, const JointNameToValueMap& joint_velocities,
    const JointNameToValueMap& joint_accelerations) const
{
    std::vector<RobotNode::SharedPtr> joint_nodes;
    for (const auto& relative_joint_node : m_relative_joint_nodes) {
        joint_nodes.push_back(relative_joint_node.lock());
    }
    Eigen::MatrixXd dynamical_torque_matrix = evaluateExpression(m_dynamical_torque_expressions, joint_nodes,
        m_relative_joint_nodes.size(), 1, joint_positions, joint_velocities, joint_accelerations);
    return Eigen::Map<Eigen::VectorXd>(dynamical_torque_matrix.data(), dynamical_torque_matrix.size());
}

double RobotNode::getDynamicalJointAcceleration(double joint_torque, const JointNameToValueMap& joint_positions) const
{
    std::vector<RobotNode::SharedPtr> joint_nodes;
    for (const auto& relative_joint_node : m_relative_joint_nodes) {
        joint_nodes.push_back(relative_joint_node.lock());
    }
    double relative_inertia
        = evaluateExpression({ m_relative_inertia_expression }, joint_nodes, 1, 1, joint_positions)(0, 0);
    return joint_torque / relative_inertia;
}

void RobotNode::setExpression(const std::vector<std::string>& expressions, std::vector<GiNaC::ex>& target)
{
    if (target.size() > 0) {
        target.clear();
    }

    for (unsigned long int i = 0; i < expressions.size(); i++) {
        GiNaC::ex expression(expressions[i], m_joint_symbols_list);
        target.push_back(expression);
    }
}

Eigen::MatrixXd RobotNode::substituteExpression(const std::vector<GiNaC::ex>& expressions, const unsigned int& rows,
    const unsigned int& cols, const GiNaC::lst& substitutions) const
{
    Eigen::MatrixXd evaluation_matrix = Eigen::MatrixXd::Zero(rows, cols);
    for (unsigned int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < cols; j++) {
            GiNaC::ex expression = GiNaC::evalf(expressions[i * cols + j].subs(substitutions));
            evaluation_matrix(i, j) = GiNaC::ex_to<GiNaC::numeric>(expression).to_double();
        }
    }
    return evaluation_matrix;
}

Eigen::MatrixXd RobotNode::evaluateExpression(const std::vector<GiNaC::ex>& expressions,
    const std::vector<RobotNode::SharedPtr> joint_nodes, const unsigned int& rows, const unsigned int& cols,
    const JointNameToValueMap& joint_positions) const
{
    GiNaC::lst substitutions = GiNaC::lst();
    for (const auto& joint_node : joint_nodes) {
        substitutions.append(joint_node->getJointPosition() == joint_positions.at(joint_node->getName()));
    }
    return substituteExpression(expressions, rows, cols, substitutions);
}

Eigen::MatrixXd RobotNode::evaluateExpression(const std::vector<GiNaC::ex>& expressions,
    const std::vector<RobotNode::SharedPtr> joint_nodes, const unsigned int& rows, const unsigned int& cols,
    const JointNameToValueMap& joint_positions, const JointNameToValueMap& joint_velocities) const
{
    GiNaC::lst substitutions = GiNaC::lst();
    for (const auto& joint_node : joint_nodes) {
        substitutions.append(joint_node->getJointPosition() == joint_positions.at(joint_node->getName()));
        substitutions.append(joint_node->getJointVelocity() == joint_velocities.at(joint_node->getName()));
    }
    return substituteExpression(expressions, rows, cols, substitutions);
}

Eigen::MatrixXd RobotNode::evaluateExpression(const std::vector<GiNaC::ex>& expressions,
    const std::vector<RobotNode::SharedPtr> joint_nodes, const unsigned int& rows, const unsigned int& cols,
    const JointNameToValueMap& joint_positions, const JointNameToValueMap& joint_velocities,
    const JointNameToValueMap& joint_accelerations) const
{
    GiNaC::lst substitutions = GiNaC::lst();
    for (const auto& joint_node : joint_nodes) {
        substitutions.append(joint_node->getJointPosition() == joint_positions.at(joint_node->getName()));
        substitutions.append(joint_node->getJointVelocity() == joint_velocities.at(joint_node->getName()));
        substitutions.append(joint_node->getJointAcceleration() == joint_accelerations.at(joint_node->getName()));
    }
    return substituteExpression(expressions, rows, cols, substitutions);
}