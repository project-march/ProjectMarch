#include "march_state_estimator/robot_joint.hpp"

RobotJoint::RobotJoint(const std::string & name, const uint64_t & id, const std::vector<double> & axis)
{
    name_ = name;
    id_ = id;
    type_ = 'J';

    axis_ = axis;
    joint_angle_ = GiNaC::symbol("q_{" + name + "}");
}

void RobotJoint::setLimits(const double & lower_limit, const double & upper_limit)
{
    lower_limit_ = lower_limit;
    upper_limit_ = upper_limit;
}

void RobotJoint::setOriginRotation(const Eigen::Matrix3d & rotation)
{
    GiNaC::matrix origin_rotation_matrix = utilConvertEigenToGiNaC(rotation);
    GiNaC::matrix angle_rotation_matrix = utilRotate(axis_);
    origin_rotation_matrix_ = angle_rotation_matrix.mul(origin_rotation_matrix);
}

GiNaC::matrix RobotJoint::utilRotate(std::vector<double> & axis)
{
    GiNaC::matrix rotation_matrix(WORKSPACE_DIM, WORKSPACE_DIM);
    rotation_matrix = utilRotateX(axis[0] * joint_angle_);
    rotation_matrix = utilRotateY(axis[1] * joint_angle_).mul(rotation_matrix);
    rotation_matrix = utilRotateZ(axis[2] * joint_angle_).mul(rotation_matrix);
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