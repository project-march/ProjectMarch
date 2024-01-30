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

std::vector<double> RobotNode::getJointAxis() const
{
    return m_joint_axis;
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

Eigen::Vector3d RobotNode::getGlobalPosition(JointNameToValueMap joint_positions) const
{
    return Eigen::Map<Eigen::Vector3d>(
        evaluateExpression(m_global_position_expressions, joint_positions, WORKSPACE_DIM, 1).data());
}

Eigen::Matrix3d RobotNode::getGlobalRotation(JointNameToValueMap joint_positions) const
{
    return Eigen::Map<Eigen::Matrix3d>(
        evaluateExpression(m_global_rotation_expressions, joint_positions, WORKSPACE_DIM, WORKSPACE_DIM).data());
}

Eigen::MatrixXd RobotNode::getGlobalPositionJacobian(JointNameToValueMap joint_positions) const
{
    return evaluateExpression(
        m_global_position_jacobian_expressions, joint_positions, WORKSPACE_DIM, m_joint_nodes.size());
}

Eigen::MatrixXd RobotNode::getGlobalRotationJacobian(JointNameToValueMap joint_positions) const
{
    (void)joint_positions;
    return Eigen::MatrixXd::Zero(WORKSPACE_DIM, m_joint_nodes.size());
}

Eigen::VectorXd RobotNode::getDynamicalTorque(JointNameToValueMap joint_positions, JointNameToValueMap joint_velocities,
    JointNameToValueMap joint_accelerations) const
{
    GiNaC::lst substitutions = GiNaC::lst();
    for (const auto& relative_joint_node : m_relative_joint_nodes) {
        RobotNode::SharedPtr joint_node = relative_joint_node.lock();
        substitutions.append(joint_node->getJointPosition() == joint_positions.at(joint_node->getName()));
        substitutions.append(joint_node->getJointVelocity() == joint_velocities.at(joint_node->getName()));
        substitutions.append(joint_node->getJointAcceleration() == joint_accelerations.at(joint_node->getName()));
    }
    Eigen::VectorXd evaluation_matrix = Eigen::VectorXd::Zero(m_relative_joint_nodes.size());
    for (unsigned int i = 0; i < m_relative_joint_nodes.size(); i++) {
        GiNaC::ex expression = GiNaC::evalf(m_dynamical_torque_expressions[i].subs(substitutions));
        evaluation_matrix(i) = GiNaC::ex_to<GiNaC::numeric>(expression).to_double();
    }
    return evaluation_matrix;
}

double RobotNode::getDynamicalJointAcceleration(double joint_torque, JointNameToValueMap joint_positions) const
{
    // TODO: Create a relative substitution function.
    GiNaC::lst substitutions = GiNaC::lst();
    for (const auto& relative_joint_node : m_relative_joint_nodes) {
        RobotNode::SharedPtr joint_node = relative_joint_node.lock();
        substitutions.append(joint_node->getJointPosition() == joint_positions.at(joint_node->getName()));
    }
    GiNaC::ex expression = GiNaC::evalf(m_relative_inertia_expression.subs(substitutions));
    double relative_inertia = GiNaC::ex_to<GiNaC::numeric>(expression).to_double();
    return joint_torque / relative_inertia;
}

std::vector<RobotNode::SharedPtr> RobotNode::getJointNodes(RobotNode::SharedPtr parent) const
{
    std::vector<RobotNode::SharedPtr> joint_nodes;
    while (parent != nullptr) {
        if (parent->getType() == 'J') {
            joint_nodes.push_back(parent);
        }
        parent = parent->getParent();
    }
    std::reverse(joint_nodes.begin(), joint_nodes.end());
    return joint_nodes;
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

Eigen::MatrixXd RobotNode::evaluateExpression(const std::vector<GiNaC::ex>& expressions,
    const JointNameToValueMap& joint_positions, const unsigned int& rows, const unsigned int& cols) const
{
    GiNaC::lst substitutions = substituteSymbolsWithJointValues(joint_positions);
    Eigen::MatrixXd evaluation_matrix = Eigen::MatrixXd::Zero(rows, cols);
    for (unsigned int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < cols; j++) {
            GiNaC::ex expression = GiNaC::evalf(expressions[i * cols + j].subs(substitutions));
            evaluation_matrix(i, j) = GiNaC::ex_to<GiNaC::numeric>(expression).to_double();
        }
    }
    return evaluation_matrix;
}

GiNaC::lst RobotNode::substituteSymbolsWithJointValues(const JointNameToValueMap& joint_positions) const
{
    GiNaC::lst substitutions = GiNaC::lst();
    for (const auto& joint_node : m_joint_nodes) {
        substitutions.append(joint_node->getJointPosition() == joint_positions.at(joint_node->getName()));
    }
    return substitutions;
}

int RobotNode::utilGetJointAxisIndex() const
{
    // Assuming that the joint axis is orthogonal to the other two axes.
    int joint_axis_index = -1;
    for (long unsigned int i = 0; i < m_joint_axis.size(); i++) {
        if (m_joint_axis[i] != 0) {
            joint_axis_index = i;
            break;
        }
    }
    return joint_axis_index;
}