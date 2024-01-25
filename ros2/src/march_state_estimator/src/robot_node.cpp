/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <algorithm>
#include "math.h"

void RobotNode::setName(const std::string & name)
{
    m_name = name;
}

void RobotNode::setId(const uint64_t & id)
{
    m_id = id;
}

void RobotNode::setParent(std::shared_ptr<RobotNode> parent)
{
    m_parent = parent;
}

void RobotNode::addChild(std::shared_ptr<RobotNode> child)
{
    m_children.push_back(child);
    child->setParent(shared_from_this());
}

void RobotNode::setJointNodes(std::vector<std::shared_ptr<RobotNode>> joint_nodes)
{
    m_joint_nodes = joint_nodes;
    for (auto & joint_node : m_joint_nodes)
    {
        m_joint_symbols_list.append(joint_node->getJointAngle());
    }
}

void RobotNode::setOriginPosition(const Eigen::Vector3d & position)
{
    Eigen::MatrixXd position_matrix(WORKSPACE_DIM, 1);
    position_matrix = position;
    m_origin_position_vector = utilConvertEigenToGiNaC(position_matrix);
}

void RobotNode::setOriginRotation(const Eigen::Matrix3d & rotation)
{
    m_origin_rotation_matrix = utilConvertEigenToGiNaC(rotation);
}

// TODO: Convert these functions into typedefs?
void RobotNode::setExpressionGlobalPosition(const std::vector<std::string> & expressions)
{
    setExpression(expressions, m_global_position_expressions);
}

void RobotNode::setExpressionGlobalRotation(const std::vector<std::string> & expressions)
{
    setExpression(expressions, m_global_rotation_expressions);
}

void RobotNode::setExpressionGlobalPositionJacobian(const std::vector<std::string> & expressions)
{
    setExpression(expressions, m_global_position_jacobian_expressions);
}

void RobotNode::setExpressionGlobalRotationJacobian(const std::vector<std::string> & expressions)
{
    setExpression(expressions, m_global_rotation_jacobian_expressions);
}

std::string RobotNode::getName() const
{
    return m_name;
}

uint64_t RobotNode::getId() const
{
    return m_id;
}

uint64_t RobotNode::getId(const std::string & name) const
{
    return std::hash<std::string>{}(name);
}

char RobotNode::getType() const
{
    return m_type;
}

std::vector<double> RobotNode::getJointAxis() const
{
    return m_joint_axis;
}

GiNaC::matrix RobotNode::getOriginPosition() const
{
    return m_origin_position_vector;
}

GiNaC::matrix RobotNode::getOriginRotation() const
{
    return m_origin_rotation_matrix;
}

GiNaC::symbol RobotNode::getJointAngle() const
{
    return m_joint_angle;
}

std::shared_ptr<RobotNode> RobotNode::getParent() const
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
    for (auto & joint_node : m_joint_nodes)
    {
        joint_names.push_back(joint_node->getName());
    }
    return joint_names;
}

GiNaC::matrix RobotNode::getGlobalPositionExpression() const
{
    return m_global_position_vector;
}

GiNaC::matrix RobotNode::getGlobalRotationExpression() const
{
    return m_global_rotation_matrix;
}

Eigen::Vector3d RobotNode::getGlobalPosition(std::unordered_map<std::string, double> joint_positions) const
{
    return Eigen::Map<Eigen::Vector3d>(evaluateExpression(m_global_position_expressions, joint_positions, WORKSPACE_DIM, 1).data());
}

Eigen::Matrix3d RobotNode::getGlobalRotation(std::unordered_map<std::string, double> joint_positions) const
{
    return Eigen::Map<Eigen::Matrix3d>(evaluateExpression(m_global_rotation_expressions, joint_positions, WORKSPACE_DIM, WORKSPACE_DIM).data());
}

Eigen::MatrixXd RobotNode::getGlobalPositionJacobian(std::unordered_map<std::string, double> joint_positions) const
{
    return evaluateExpression(m_global_position_jacobian_expressions, joint_positions, WORKSPACE_DIM, m_joint_nodes.size());
}

Eigen::MatrixXd RobotNode::getGlobalRotationJacobian(std::unordered_map<std::string, double> joint_positions) const
{
    //  TODO: To optimize -> Override in RobotMass.
    if (m_type == 'M')
    {
        return Eigen::MatrixXd::Zero(WORKSPACE_DIM, m_joint_nodes.size());
    }

    return evaluateExpression(m_global_rotation_jacobian_expressions, joint_positions, WORKSPACE_DIM, m_joint_nodes.size());
}

std::vector<GiNaC::symbol> RobotNode::getJointAngles() const
{
    std::vector<GiNaC::symbol> joint_angles;
    std::shared_ptr<RobotNode> parent = m_parent;
    while (parent != nullptr)
    {
        if (parent->getType() == 'J')
        {
            joint_angles.push_back(parent->getJointAngle());
        }
        parent = parent->getParent();
    }
    return joint_angles;
}

std::vector<std::shared_ptr<RobotNode>> RobotNode::getJointNodes(std::shared_ptr<RobotNode> parent) const
{
    std::vector<std::shared_ptr<RobotNode>> joint_nodes;
    while (parent != nullptr)
    {
        if (parent->getType() == 'J')
        {
            joint_nodes.push_back(parent);
        }
        parent = parent->getParent();
    }
    std::reverse(joint_nodes.begin(), joint_nodes.end());
    return joint_nodes;
}

void RobotNode::expressRotation()
{
    m_global_rotation_matrix = expressGlobalRotation();

    // DEBUG: Print the global rotation expressions.
    for (unsigned int i = 0; i < m_global_rotation_matrix.rows(); i++)
    {
        for (unsigned int j = 0; j < m_global_rotation_matrix.cols(); j++)
        {
            std::stringstream ss_rotation_expression;
            ss_rotation_expression << "Global rotation " << m_name << " (" << i << "," << j << ") : " << m_global_rotation_matrix(i, j) << " ";
            RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), ss_rotation_expression.str().c_str());
        }
    }
}

void RobotNode::expressKinematics()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotNode::expressKinematics: %s", m_name.c_str());
    m_global_rotation_matrix = expressGlobalRotation();
    m_global_position_vector = expressGlobalPosition();

    m_joint_nodes = getJointNodes(shared_from_this());
    m_global_position_jacobian_matrix = expressGlobalPositionJacobian();
    m_global_rotation_jacobian_matrix = expressGlobalRotationJacobian();

    // DEBUG: Print the global position expressions.
    for (unsigned int i = 0; i < m_global_position_vector.rows(); i++)
    {
        std::stringstream ss_position_expression;
        ss_position_expression << "Global position " << m_name << " " << i << ": " << m_global_position_vector(i, 0) << " ";
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), ss_position_expression.str().c_str());
    }
}

GiNaC::matrix RobotNode::expressGlobalPosition() const
{
    GiNaC::matrix global_position = m_origin_position_vector;
    if (m_parent != nullptr)
    {
        GiNaC::matrix rotated_origin_position = m_parent->getGlobalRotationExpression().mul(global_position);
        global_position = rotated_origin_position.add(m_parent->expressGlobalPosition());
    }
    return global_position;
}

GiNaC::matrix RobotNode::expressGlobalRotation() const
{
    GiNaC::matrix global_rotation = getOriginRotation();
    if (m_parent != nullptr)
    {
        global_rotation = global_rotation.mul(m_parent->expressGlobalRotation());
    }
    return global_rotation;
}

GiNaC::matrix RobotNode::expressGlobalPositionJacobian() const
{
    GiNaC::matrix global_position_jacobian(WORKSPACE_DIM, m_joint_nodes.size());

    // TODO: Replace this into a util function.
    for (int i = 0; i < WORKSPACE_DIM; i++)
    {
        for (long unsigned int j = 0; j < m_joint_nodes.size(); j++)
        {
            GiNaC::ex global_position_component = m_global_position_vector(i, 0).diff(m_joint_nodes[j]->getJointAngle());
            global_position_jacobian(i, j) = global_position_component;
        }
    }
    return global_position_jacobian;
}

GiNaC::matrix RobotNode::expressGlobalRotationJacobian() const
{
    GiNaC::matrix global_rotation_jacobian(WORKSPACE_DIM, m_joint_nodes.size());

    if (m_type == 'J')
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotNode::expressGlobalRotationJacobian: m_type == 'J'");
        // TODO: Replace this into a util function.
        for (long unsigned int i = 0; i < WORKSPACE_DIM; i++)
        {
            for (long unsigned int j = 0; j < m_joint_nodes.size(); j++)
            {
                GiNaC::ex global_rotation_component = m_global_rotation_matrix(i, utilGetJointAxisIndex()).diff(m_joint_nodes[j]->getJointAngle());
                global_rotation_jacobian(i, j) = global_rotation_component;
            }
        }
    }
    else // m_type == 'M'
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotNode::expressGlobalRotationJacobian: m_type == 'M'");
        for (long unsigned int i = 0; i < WORKSPACE_DIM; i++)
        {
            for (long unsigned int j = 0; j < m_joint_nodes.size(); j++)
            {
                global_rotation_jacobian(i, j) = 0;
            }
        }
    }

    return global_rotation_jacobian;
}

void RobotNode::setExpression(const std::vector<std::string> & expressions, std::vector<GiNaC::ex> & target)
{
    if (target.size() > 0)
    {
        target.clear();
    }

    for (unsigned long int i = 0; i < expressions.size(); i++)
    {
        GiNaC::ex expression(expressions[i], m_joint_symbols_list);
        target.push_back(expression);
    }
}

Eigen::MatrixXd RobotNode::evaluateExpression(const std::vector<GiNaC::ex> & expressions, 
        const std::unordered_map<std::string, double> & joint_positions,
        const unsigned int & rows, const unsigned int & cols) const
{
    GiNaC::lst substitutions = substituteSymbolsWithJointValues(joint_positions);
    Eigen::MatrixXd evaluation_matrix = Eigen::MatrixXd::Zero(rows, cols);
    for (unsigned int i = 0; i < rows; i++)
    {
        for (unsigned int j = 0; j < cols; j++)
        {
            GiNaC::ex expression = GiNaC::evalf(expressions[i * cols + j].subs(substitutions));
            evaluation_matrix(i, j) = GiNaC::ex_to<GiNaC::numeric>(expression).to_double();
        }
    }
    return evaluation_matrix;
}

GiNaC::lst RobotNode::substituteSymbolsWithJointValues(const std::unordered_map<std::string, double> & joint_positions) const
{
    GiNaC::lst substitutions = GiNaC::lst();
    for (const auto & joint_node : m_joint_nodes)
    {
        substitutions.append(joint_node->getJointAngle() == joint_positions.at(joint_node->getName()));
    }
    return substitutions;
}

GiNaC::matrix RobotNode::utilConvertEigenToGiNaC(const Eigen::MatrixXd & matrix) const
{
    GiNaC::matrix ginac_matrix(matrix.rows(), matrix.cols());
    for (unsigned int i = 0; i < matrix.rows(); i++)
    {
        for (unsigned int j = 0; j < matrix.cols(); j++)
        {
            ginac_matrix(i, j) = matrix(i, j);
        }
    }
    return ginac_matrix;
}

Eigen::Matrix3d RobotNode::utilConvertGiNaCToEigen(const GiNaC::matrix & matrix) const
{
    Eigen::Matrix3d eigen_matrix = Eigen::Matrix3d::Zero();
    for (unsigned int i = 0; i < matrix.rows(); i++)
    {
        for (unsigned int j = 0; j < matrix.cols(); j++)
        {
            eigen_matrix(i, j) = GiNaC::ex_to<GiNaC::numeric>(matrix(i, j)).to_double();
        }
    }
    return eigen_matrix;
}

int RobotNode::utilGetJointAxisIndex() const
{
    // Assuming that the joint axis is orthogonal to the other two axes.
    int joint_axis_index = -1;
    for (long unsigned int i = 0; i < m_joint_axis.size(); i++)
    {
        if (m_joint_axis[i] != 0)
        {
            joint_axis_index = i;
            break;
        }
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Joint axis index: %d", joint_axis_index);
    return joint_axis_index;
}