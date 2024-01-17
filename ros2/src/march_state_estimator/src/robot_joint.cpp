#include "march_state_estimator/robot_joint.hpp"

#include "rclcpp/rclcpp.hpp"

RobotJoint::RobotJoint(const std::string & name, const uint64_t & id, const std::vector<double> & axis)
{
    m_name = name;
    m_id = id;
    m_type = 'J';

    m_joint_axis = axis;
    m_joint_angle = GiNaC::symbol("q_{" + name + "}");
}

void RobotJoint::setLimits(const double & lower_limit, const double & upper_limit)
{
    m_lower_limit = lower_limit;
    m_upper_limit = upper_limit;
}

void RobotJoint::setOriginRotation(const Eigen::Matrix3d & rotation)
{
    GiNaC::matrix origin_rotation_matrix = utilConvertEigenToGiNaC(rotation);
    // GiNaC::matrix angle_rotation_matrix = utilRotate(m_joint_axis);
    // m_origin_rotation_matrix = angle_rotation_matrix.mul(origin_rotation_matrix);
    m_origin_rotation_matrix = utilRotate(m_joint_axis);

    // std::stringstream ss;

    // ss << "Origin rotation matrix for joint " << name_ << ":\n";
    // ss << origin_rotation_matrix(0, 0) << " " << origin_rotation_matrix(0, 1) << " " << origin_rotation_matrix(0, 2) << "\n";
    // ss << origin_rotation_matrix(1, 0) << " " << origin_rotation_matrix(1, 1) << " " << origin_rotation_matrix(1, 2) << "\n";
    // ss << origin_rotation_matrix(2, 0) << " " << origin_rotation_matrix(2, 1) << " " << origin_rotation_matrix(2, 2) << "\n";
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());

    // ss.str("");
    // ss << "Angle rotation matrix for joint " << name_ << ":\n";
    // ss << angle_rotation_matrix(0, 0) << " " << angle_rotation_matrix(0, 1) << " " << angle_rotation_matrix(0, 2) << "\n";
    // ss << angle_rotation_matrix(1, 0) << " " << angle_rotation_matrix(1, 1) << " " << angle_rotation_matrix(1, 2) << "\n";
    // ss << angle_rotation_matrix(2, 0) << " " << angle_rotation_matrix(2, 1) << " " << angle_rotation_matrix(2, 2) << "\n";
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());

    // ss.str("");
    // ss << "Joint rotation matrix for joint " << name_ << ":\n";
    // ss << origin_rotation_matrix_(0, 0) << " " << origin_rotation_matrix_(0, 1) << " " << origin_rotation_matrix_(0, 2) << "\n";
    // ss << origin_rotation_matrix_(1, 0) << " " << origin_rotation_matrix_(1, 1) << " " << origin_rotation_matrix_(1, 2) << "\n";
    // ss << origin_rotation_matrix_(2, 0) << " " << origin_rotation_matrix_(2, 1) << " " << origin_rotation_matrix_(2, 2) << "\n";
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());
}

GiNaC::matrix RobotJoint::utilRotate(std::vector<double> & axis)
{
    GiNaC::matrix rotation_matrix(WORKSPACE_DIM, WORKSPACE_DIM);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotJoint name: %s", m_name.c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotJoint::utilRotate: Axis: %f %f %f", axis[0], axis[1], axis[2]);
    // switch (utilGetJointAxisIndex())
    // {
    //     case 0:
    //         rotation_matrix = utilRotateX(axis[0] * m_joint_angle);
    //         break;
    //     case 1:
    //         rotation_matrix = utilRotateY(axis[1] * m_joint_angle);
    //         break;
    //     case 2:
    //         rotation_matrix = utilRotateZ(axis[2] * m_joint_angle);
    //         break;
    //     default:
    //         rotation_matrix = utilIdentity();
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RobotJoint::utilRotate: Invalid joint axis index");
    //         break;
    // }
    rotation_matrix = utilRotateX(axis[0] * m_joint_angle);
    rotation_matrix = utilRotateY(axis[1] * m_joint_angle).mul(rotation_matrix);
    rotation_matrix = utilRotateZ(axis[2] * m_joint_angle).mul(rotation_matrix);

    // for (uint64_t i = 0; i < WORKSPACE_DIM; i++)
    // {
    //     std::stringstream ss;
    //     ss << "RobotJoint::utilRotate: Rotation matrix: ";
    //     for (uint64_t j = 0; j < WORKSPACE_DIM; j++)
    //     {
    //         ss << rotation_matrix(i, j) << " ";
    //     }
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), ss.str().c_str());
    // }

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