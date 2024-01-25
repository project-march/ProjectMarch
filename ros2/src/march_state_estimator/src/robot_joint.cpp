/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_joint.hpp"

#include "rclcpp/rclcpp.hpp"

RobotJoint::RobotJoint(const std::string & name, const uint64_t & id, const std::vector<double> & axis)
{
    m_name = name;
    m_id = id;
    m_type = 'J';

    m_joint_axis = axis;
    m_joint_angle = GiNaC::symbol("q_" + name);
}

void RobotJoint::setLimits(const double & lower_limit, const double & upper_limit)
{
    m_lower_limit = lower_limit;
    m_upper_limit = upper_limit;
}

void RobotJoint::setOriginRotation(const Eigen::Matrix3d & rotation)
{
    GiNaC::matrix origin_rotation_matrix = utilConvertEigenToGiNaC(rotation);
    m_origin_rotation_matrix = utilRotate(m_joint_axis);
}

GiNaC::matrix RobotJoint::utilRotate(std::vector<double> & axis)
{
    GiNaC::matrix rotation_matrix(WORKSPACE_DIM, WORKSPACE_DIM);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotJoint name: %s", m_name.c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotJoint::utilRotate: Axis: %f %f %f", axis[0], axis[1], axis[2]);

    rotation_matrix = utilRotateX(axis[0] * m_joint_angle);
    rotation_matrix = utilRotateY(axis[1] * m_joint_angle).mul(rotation_matrix);
    rotation_matrix = utilRotateZ(axis[2] * m_joint_angle).mul(rotation_matrix);

    return rotation_matrix;
}

GiNaC::matrix RobotJoint::utilRotateX(const GiNaC::ex & angle) const
{
    GiNaC::matrix rotation_matrix(WORKSPACE_DIM, WORKSPACE_DIM);
    rotation_matrix(0, 0) = 1;
    rotation_matrix(0, 1) = 0;
    rotation_matrix(0, 2) = 0;
    rotation_matrix(1, 0) = 0;
    rotation_matrix(1, 1) = cos(angle);
    rotation_matrix(1, 2) = -sin(angle);
    rotation_matrix(2, 0) = 0;
    rotation_matrix(2, 1) = sin(angle);
    rotation_matrix(2, 2) = cos(angle);

    return rotation_matrix;
}

GiNaC::matrix RobotJoint::utilRotateY(const GiNaC::ex & angle) const
{
    GiNaC::matrix rotation_matrix(WORKSPACE_DIM, WORKSPACE_DIM);
    rotation_matrix(0, 0) = cos(angle);
    rotation_matrix(0, 1) = 0;
    rotation_matrix(0, 2) = sin(angle);
    rotation_matrix(1, 0) = 0;
    rotation_matrix(1, 1) = 1;
    rotation_matrix(1, 2) = 0;
    rotation_matrix(2, 0) = -sin(angle);
    rotation_matrix(2, 1) = 0;
    rotation_matrix(2, 2) = cos(angle);

    return rotation_matrix;
}

GiNaC::matrix RobotJoint::utilRotateZ(const GiNaC::ex & angle) const
{
    GiNaC::matrix rotation_matrix(WORKSPACE_DIM, WORKSPACE_DIM);
    rotation_matrix(0, 0) = cos(angle);
    rotation_matrix(0, 1) = -sin(angle);
    rotation_matrix(0, 2) = 0;
    rotation_matrix(1, 0) = sin(angle);
    rotation_matrix(1, 1) = cos(angle);
    rotation_matrix(1, 2) = 0;
    rotation_matrix(2, 0) = 0;
    rotation_matrix(2, 1) = 0;
    rotation_matrix(2, 2) = 1;

    return rotation_matrix;
}

GiNaC::matrix RobotJoint::utilIdentity() const
{
    GiNaC::matrix identity_matrix(WORKSPACE_DIM, WORKSPACE_DIM);
    identity_matrix(0, 0) = 1;
    identity_matrix(0, 1) = 0;
    identity_matrix(0, 2) = 0;
    identity_matrix(1, 0) = 0;
    identity_matrix(1, 1) = 1;
    identity_matrix(1, 2) = 0;
    identity_matrix(2, 0) = 0;
    identity_matrix(2, 1) = 0;
    identity_matrix(2, 2) = 1;

    return identity_matrix;
}