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

void RobotNode::setParent(RobotNode * parent)
{
    m_parent = parent;
}

void RobotNode::addChild(RobotNode * child)
{
    m_children.push_back(child);
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

RobotNode * RobotNode::getParent() const
{
    return m_parent;
}

std::vector<RobotNode*> RobotNode::getChildren() const
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

Eigen::Vector3d RobotNode::getGlobalPosition(std::vector<std::string> joint_names, std::vector<double> joint_angles) const
{
    GiNaC::lst substitutions = GiNaC::lst();

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Global position node name: %s", m_name.c_str());

    // TODO: Optimize this using std::find.
    for (auto & joint_node : m_joint_nodes)
    {
        for (long unsigned int i = 0; i < joint_names.size(); i++)
        {
            if (joint_node->getName() == joint_names[i])
            {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Joint name: %s", joint_node->getName().c_str());
                substitutions.append(joint_node->getJointAngle() == joint_angles[i]);
                break;
            }
        }
    }

    Eigen::Vector3d global_position = Eigen::Vector3d::Zero();
    for (int i = 0; i < WORKSPACE_DIM; i++)
    {
        std::stringstream ss;
        ss << i << ": " << m_global_position_vector(i, 0);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());
        GiNaC::ex global_position_component = GiNaC::evalf(m_global_position_vector(i, 0).subs(substitutions));
        global_position(i) = GiNaC::ex_to<GiNaC::numeric>(global_position_component).to_double();
    }

    return global_position;
}

Eigen::Matrix3d RobotNode::getGlobalRotation(std::vector<std::string> joint_names, std::vector<double> joint_angles) const
{
    // TODO: Optimize this using std::find.
    GiNaC::lst substitutions = GiNaC::lst();

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Global rotation node name: %s", m_name.c_str());

    // TODO: Utilize this into a function.
    for (auto & joint_node : m_joint_nodes)
    {
        for (long unsigned int i = 0; i < joint_names.size(); i++)
        {
            if (joint_node->getName() == joint_names[i])
            {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Joint name: %s", joint_node->getName().c_str());
                substitutions.append(joint_node->getJointAngle() == joint_angles[i]);
                break;
            }
        }
    }

    Eigen::Matrix3d global_rotation = Eigen::Matrix3d::Zero();
    for (int i = 0; i < WORKSPACE_DIM; i++)
    {
        for (int j = 0; j < WORKSPACE_DIM; j++)
        {
            // std::stringstream ss;
            // ss << "(" << i << "," << j << "): " << m_global_rotation_matrix(i, j);
            // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());
            GiNaC::ex global_rotation_component = GiNaC::evalf(m_global_rotation_matrix(i, j).subs(substitutions));
            global_rotation(i, j) = GiNaC::ex_to<GiNaC::numeric>(global_rotation_component).to_double();
        }
    }

    return global_rotation;
}

Eigen::MatrixXd RobotNode::getGlobalPositionJacobian(std::vector<std::string> joint_names, std::vector<double> joint_angles) const
{
    GiNaC::lst substitutions = GiNaC::lst();
    for (auto & joint_node : m_joint_nodes)
    {
        for (long unsigned int i = 0; i < joint_names.size(); i++)
        {
            if (joint_node->getName() == joint_names[i])
            {
                substitutions.append(joint_node->getJointAngle() == joint_angles[i]);
                break;
            }
        }
    }

    Eigen::MatrixXd global_position_jacobian = Eigen::MatrixXd::Zero(WORKSPACE_DIM, m_joint_nodes.size());
    for (int i = 0; i < WORKSPACE_DIM; i++)
    {
        for (long unsigned int j = 0; j < m_joint_nodes.size(); j++)
        {
            GiNaC::ex global_position_jacobian_component = GiNaC::evalf(m_global_position_jacobian_matrix(i, j).subs(substitutions));
            global_position_jacobian(i, j) = GiNaC::ex_to<GiNaC::numeric>(global_position_jacobian_component).to_double();
        }
    }

    return global_position_jacobian;
}

Eigen::MatrixXd RobotNode::getGlobalRotationJacobian(std::vector<std::string> joint_names, std::vector<double> joint_angles) const
{
    //  TODO: To optimize
    if (m_type == 'M')
    {
        return Eigen::MatrixXd::Zero(WORKSPACE_DIM, m_joint_nodes.size());
    }

    GiNaC::lst substitutions = GiNaC::lst();
    for (auto & joint_node : m_joint_nodes)
    {
        for (long unsigned int i = 0; i < joint_names.size(); i++)
        {
            if (joint_node->getName() == joint_names[i])
            {
                substitutions.append(joint_node->getJointAngle() == joint_angles[i]);
                break;
            }
        }
    }

    Eigen::MatrixXd global_rotation_jacobian = Eigen::MatrixXd::Zero(WORKSPACE_DIM, m_joint_nodes.size());
    for (unsigned int i = 0; i < m_global_rotation_jacobian_matrix.rows(); i++)
    {
        for (unsigned int j = 0; j < m_global_rotation_jacobian_matrix.cols(); j++)
        {
            GiNaC::ex global_rotation_jacobian_component = GiNaC::evalf(m_global_rotation_jacobian_matrix(i, j).subs(substitutions));
            global_rotation_jacobian(i, j) = GiNaC::ex_to<GiNaC::numeric>(global_rotation_jacobian_component).to_double();
        }
    }

    return global_rotation_jacobian;
}

std::vector<GiNaC::symbol> RobotNode::getJointAngles() const
{
    std::vector<GiNaC::symbol> joint_angles;
    RobotNode * parent = m_parent;
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

std::vector<RobotNode*> RobotNode::getJointNodes(RobotNode * parent) const
{
    std::vector<RobotNode*> joint_nodes;
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

void RobotNode::expressKinematics()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotNode::expressKinematics: %s", m_name.c_str());
    GiNaC::matrix global_position = expressGlobalPosition();
    GiNaC::matrix global_rotation = expressGlobalRotation();

    for (unsigned int i = 0; i < global_position.rows(); i++)
    {
        global_position(i, 0) = global_position(i, 0).simplify_indexed();
    }
    for (unsigned int i = 0; i < global_rotation.rows(); i++)
    {
        for (unsigned int j = 0; j < global_rotation.cols(); j++)
        {
            global_rotation(i, j) = global_rotation(i, j).simplify_indexed();
        }
    }

    // joint_angles_ = getJointAngles();
    m_joint_nodes = getJointNodes(this);
    m_global_position_vector = global_position;
    m_global_rotation_matrix = global_rotation;

    m_global_position_jacobian_matrix = expressGlobalPositionJacobian();
    m_global_rotation_jacobian_matrix = expressGlobalRotationJacobian();
}

GiNaC::matrix RobotNode::expressGlobalPosition() const
{
    GiNaC::matrix global_position(WORKSPACE_DIM, 1);
    global_position = getOriginRotation().mul(m_origin_position_vector);
    RobotNode * parent = m_parent;
    while (parent != nullptr)
    {
        global_position = parent->getOriginRotation().mul(global_position);
        global_position = global_position.add(parent->getOriginPosition());

        parent = parent->getParent();
    }
    return global_position;
}

GiNaC::matrix RobotNode::expressGlobalRotation() const
{
    GiNaC::matrix global_rotation(WORKSPACE_DIM, WORKSPACE_DIM);
    global_rotation = m_origin_rotation_matrix;
    RobotNode * parent = m_parent;
    while (parent != nullptr)
    {
        global_rotation = parent->getOriginRotation().mul(global_rotation);

        parent = parent->getParent();
    }

    for (unsigned int i = 0; i < global_rotation.rows(); i++)
    {
        std::stringstream ss;
        ss << "Global rotation: ";
        for (unsigned int j = 0; j < global_rotation.cols(); j++)
        {
            ss << global_rotation(i, j) << " ";
        }
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());
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