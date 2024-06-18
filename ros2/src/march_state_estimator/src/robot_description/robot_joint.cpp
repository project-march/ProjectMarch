/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description/robot_joint.hpp"

#include "rclcpp/rclcpp.hpp"

RobotJoint::RobotJoint(const std::string& name, const uint64_t& id, const std::vector<double>& joint_axis)
{
    m_name = name;
    m_id = id;
    m_type = 'J';

    m_joint_symbol_position = GiNaC::symbol("q_" + name);
    m_joint_symbol_velocity = GiNaC::symbol("dq_" + name);
    m_joint_symbol_acceleration = GiNaC::symbol("ddq_" + name);
    m_joint_axis = joint_axis;
}

void RobotJoint::setLimits(const double& lower_limit, const double& upper_limit)
{
    m_lower_limit = lower_limit;
    m_upper_limit = upper_limit;
}

Eigen::MatrixXd RobotJoint::getGlobalRotationJacobian(JointNameToValueMap joint_positions) const
{
    return evaluateExpression(
        m_global_rotation_jacobian_expressions, m_joint_nodes, WORKSPACE_DIM, m_joint_nodes.size(), joint_positions);
}