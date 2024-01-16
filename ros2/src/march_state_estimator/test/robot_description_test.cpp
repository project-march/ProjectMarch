//
// Created by AlexanderJamesBecoy on 2023-01-10.
//

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.


#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include <sstream>

// #include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "march_state_estimator/robot_node.hpp"
#include "march_state_estimator/robot_description.hpp"

class RobotDescriptionTest : public ::testing::Test
{
public:
    RobotDescriptionTest() = default;
    ~RobotDescriptionTest() override = default;

protected:
    void SetUp() override
    {
        this->m_robot_description = std::make_unique<RobotDescription>();
        std::string m_urdf_path = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march9/double_rotational_test_setup.urdf";
        this->m_robot_description->parseURDF(m_urdf_path);
        this->m_robot_description->configureRobotNodes();
    }

    void TearDown() override
    {
        this->m_robot_description.reset();
    }

    std::string convertGinacExpressionToString(GiNaC::ex expression)
    {
        std::stringstream stringstream;
        stringstream << expression;
        return stringstream.str();
    }

    std::vector<std::string> convertGinacExpressionMatrixToStringVector(GiNaC::matrix expression_matrix)
    {
        std::vector<std::string> expression_vector;
        for (unsigned int i = 0; i < expression_matrix.rows(); i++)
        {
            for (unsigned int j = 0; j < expression_matrix.cols(); j++)
            {
                expression_vector.push_back(convertGinacExpressionToString(expression_matrix(i, j)));
            }
        }
        return expression_vector;
    }

    void assertElbowGlobalPositionGivenJointAngle(double joint_angle, Eigen::Vector3d expected_global_position)
    {
        Eigen::Vector3d actual_global_position = getElbowGlobalPositionGivenJointAngle(joint_angle);
        ASSERT_TRUE(expected_global_position.isApprox(actual_global_position, m_abs_error));
    }

    void assertElbowGlobalRotationGivenJointAngle(double joint_angle, Eigen::Matrix3d expected_global_rotation)
    {
        Eigen::Matrix3d actual_global_rotation = getElbowGlobalRotationGivenJointAngle(joint_angle);
        ASSERT_TRUE(expected_global_rotation.isApprox(actual_global_rotation, m_abs_error));
    }

    void assertElbowGlobalPositionJacobianGivenJointAngle(double joint_angle, Eigen::MatrixXd expected_global_position_jacobian)
    {
        Eigen::MatrixXd actual_global_position_jacobian = getElbowGlobalPositionJacobianGivenJointAngle(joint_angle);
        ASSERT_TRUE(expected_global_position_jacobian.isApprox(actual_global_position_jacobian, m_abs_error));
    }

    void assertElbowGlobalRotationJacobianGivenJointAngle(double joint_angle, Eigen::MatrixXd expected_global_rotation_jacobian)
    {
        Eigen::MatrixXd actual_global_rotation_jacobian = getElbowGlobalRotationJacobianGivenJointAngle(joint_angle);
        ASSERT_TRUE(expected_global_rotation_jacobian.isApprox(actual_global_rotation_jacobian, m_abs_error));
    }

    Eigen::Vector3d getElbowGlobalPositionGivenJointAngle(double joint_angle)
    {
        std::vector<std::string> node_names = {"link_elbow"};
        std::vector<std::string> joint_names = {"joint_shoulder"};
        std::vector<double> joint_angles = {joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalPosition(joint_names, joint_angles);
    }

    Eigen::Matrix3d getElbowGlobalRotationGivenJointAngle(double joint_angle)
    {
        std::vector<std::string> node_names = {"link_elbow"};
        std::vector<std::string> joint_names = {"joint_shoulder"};
        std::vector<double> joint_angles = {joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalRotation(joint_names, joint_angles);
    }

    Eigen::MatrixXd getElbowGlobalPositionJacobianGivenJointAngle(double joint_angle)
    {
        std::vector<std::string> node_names = {"link_elbow"};
        std::vector<std::string> joint_names = {"joint_shoulder"};
        std::vector<double> joint_angles = {joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalPositionJacobian(joint_names, joint_angles);
    }

    Eigen::MatrixXd getElbowGlobalRotationJacobianGivenJointAngle(double joint_angle)
    {
        std::vector<std::string> node_names = {"link_elbow"};
        std::vector<std::string> joint_names = {"joint_shoulder"};
        std::vector<double> joint_angles = {joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalRotationJacobian(joint_names, joint_angles);
    }

    void assertEndpointGlobalPositionGivenJointAngles(double shoulder_joint_angle, double elbow_joint_angle, Eigen::Vector3d expected_global_position)
    {
        Eigen::Vector3d actual_global_position = getEndpointGlobalPositionGivenJointAngles(shoulder_joint_angle, elbow_joint_angle);
        ASSERT_NEAR(expected_global_position(0), actual_global_position(0), m_abs_error);
        ASSERT_NEAR(expected_global_position(1), actual_global_position(1), m_abs_error);
        ASSERT_NEAR(expected_global_position(2), actual_global_position(2), m_abs_error);
    }

    void assertEndpointGlobalRotationGivenJointAngles(double shoulder_joint_angle, double elbow_joint_angle, Eigen::Matrix3d expected_global_rotation)
    {
        Eigen::Matrix3d actual_global_rotation = getEndpointGlobalRotationGivenJointAngles(shoulder_joint_angle, elbow_joint_angle);
        ASSERT_TRUE(expected_global_rotation.isApprox(actual_global_rotation, m_abs_error));
    }

    Eigen::Vector3d getEndpointGlobalPositionGivenJointAngles(double shoulder_joint_angle, double elbow_joint_angle)
    {
        std::vector<std::string> node_names = {"link_endpoint"};
        std::vector<std::string> joint_names = {"joint_shoulder", "joint_elbow"};
        std::vector<double> joint_angles = {shoulder_joint_angle, elbow_joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalPosition(joint_names, joint_angles);
    }

    Eigen::Matrix3d getEndpointGlobalRotationGivenJointAngles(double shoulder_joint_angle, double elbow_joint_angle)
    {
        std::vector<std::string> node_names = {"link_endpoint"};
        std::vector<std::string> joint_names = {"joint_shoulder", "joint_elbow"};
        std::vector<double> joint_angles = {shoulder_joint_angle, elbow_joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalRotation(joint_names, joint_angles);
    }

    Eigen::Matrix3d getEndpointGlobalPositionJacobianGivenJointAngles(double shoulder_joint_angle, double elbow_joint_angle)
    {
        std::vector<std::string> node_names = {"link_endpoint"};
        std::vector<std::string> joint_names = {"joint_shoulder", "joint_elbow"};
        std::vector<double> joint_angles = {shoulder_joint_angle, elbow_joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalPositionJacobian(joint_names, joint_angles);
    }

    Eigen::Matrix3d getEndpointGlobalRotationJacobianGivenJointAngles(double shoulder_joint_angle, double elbow_joint_angle)
    {
        std::vector<std::string> node_names = {"link_endpoint"};
        std::vector<std::string> joint_names = {"joint_shoulder", "joint_elbow"};
        std::vector<double> joint_angles = {shoulder_joint_angle, elbow_joint_angle};
        return m_robot_description->findNodes(node_names)[0]->getGlobalRotationJacobian(joint_names, joint_angles);
    }

    std::unique_ptr<RobotDescription> m_robot_description = std::make_unique<RobotDescription>();
    double m_abs_error = 1e-6;
};

TEST_F(RobotDescriptionTest, should_set_robot_information_from_rotational_setup_urdf)
{
    int expected_num_nodes = 5;
    std::string expected_link_1_name = "link_world";
    std::string expected_link_2_name = "link_elbow";
    std::string expected_link_3_name = "link_endpoint";
    std::string expected_joint_1_name = "joint_shoulder";
    std::string expected_joint_2_name = "joint_elbow";
    std::vector<double> expected_joint_1_axis = {0.0, -1.0, 0.0};
    std::vector<double> expected_joint_2_axis = {0.0, -1.0, 0.0};

    int actual_num_nodes = m_robot_description->getNodeNames().size();
    std::vector<std::string> actual_node_names = m_robot_description->getNodeNames();
    std::string actual_link_1_name = actual_node_names[2];
    std::string actual_link_2_name = actual_node_names[0];
    std::string actual_link_3_name = actual_node_names[1];
    std::string actual_joint_1_name = actual_node_names[4];
    std::string actual_joint_2_name = actual_node_names[3];
    std::vector<double> actual_joint_1_axis = m_robot_description->findNodes({actual_joint_1_name})[0]->getJointAxis();
    std::vector<double> actual_joint_2_axis = m_robot_description->findNodes({actual_joint_2_name})[0]->getJointAxis();

    ASSERT_EQ(expected_num_nodes, actual_num_nodes);
    ASSERT_EQ(expected_link_1_name, actual_link_1_name);
    ASSERT_EQ(expected_link_2_name, actual_link_2_name);
    ASSERT_EQ(expected_link_3_name, actual_link_3_name);
    ASSERT_EQ(expected_joint_1_name, actual_joint_1_name);
    ASSERT_EQ(expected_joint_2_name, actual_joint_2_name);
    ASSERT_EQ(expected_joint_1_axis, actual_joint_1_axis);
    ASSERT_EQ(expected_joint_2_axis, actual_joint_2_axis);
}

TEST_F(RobotDescriptionTest, should_get_expected_elbow_global_position_expression)
{
    std::vector<std::string> node_names = {"link_elbow"};
    std::string expected_global_position_expression_x = "sin(-q_{joint_shoulder})";
    std::string expected_global_position_expression_y = "0.0";
    std::string expected_global_position_expression_z = "cos(-q_{joint_shoulder})";

    GiNaC::matrix actual_global_position_expression = m_robot_description->findNodes(node_names)[0]->getGlobalPositionExpression();
    std::string actual_global_position_expression_x = convertGinacExpressionToString(actual_global_position_expression(0, 0));
    std::string actual_global_position_expression_y = convertGinacExpressionToString(actual_global_position_expression(1, 0));
    std::string actual_global_position_expression_z = convertGinacExpressionToString(actual_global_position_expression(2, 0));

    ASSERT_EQ(expected_global_position_expression_x, actual_global_position_expression_x);
    ASSERT_EQ(expected_global_position_expression_y, actual_global_position_expression_y);
    ASSERT_EQ(expected_global_position_expression_z, actual_global_position_expression_z);
}

TEST_F(RobotDescriptionTest, should_get_expected_elbow_global_rotation_expression)
{
    std::vector<std::string> node_names = {"link_elbow"};
    std::vector<std::string> expected_global_rotation_expression = {
        "cos(-q_{joint_shoulder})" ,       "0"     ,    "sin(-q_{joint_shoulder})"    , 
            "0"         ,       "1.0"   ,       "0"             , 
        "-sin(-q_{joint_shoulder})",       "0"     ,    "cos(-q_{joint_shoulder})"
    };

    GiNaC::matrix actual_global_rotation_expression_matrix = m_robot_description->findNodes(node_names)[0]->getGlobalRotationExpression();
    std::vector<std::string> actual_global_rotation_expression = convertGinacExpressionMatrixToStringVector(actual_global_rotation_expression_matrix);

    for (long unsigned int i = 0; i < expected_global_rotation_expression.size(); i++)
    {
        ASSERT_EQ(expected_global_rotation_expression[i], actual_global_rotation_expression[i]);
    }
}

TEST_F(RobotDescriptionTest, should_get_upright_elbow_global_position)
{
    double upright_joint_angle = 0.0;
    Eigen::Vector3d expected_global_position = {0.0, 0.0, 1.0};
    assertElbowGlobalPositionGivenJointAngle(upright_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_downward_elbow_global_position)
{
    double downward_joint_angle = M_PI;
    Eigen::Vector3d expected_global_position = {0.0, 0.0, -1.0};
    assertElbowGlobalPositionGivenJointAngle(downward_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_left_sideways_elbow_global_position)
{
    double left_sideways_joint_angle = M_PI_2;
    Eigen::Vector3d expected_global_position = {-1.0, 0.0, 0.0};
    assertElbowGlobalPositionGivenJointAngle(left_sideways_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_right_sideways_elbow_global_position)
{
    double right_sideways_joint_angle = -M_PI_2;
    Eigen::Vector3d expected_global_position = {1.0, 0.0, 0.0};
    assertElbowGlobalPositionGivenJointAngle(right_sideways_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_diagonal_upright_left_elbow_global_position)
{
    double diagonal_upright_left_joint_angle = M_PI_4;
    Eigen::Vector3d expected_global_position = {-0.70710678118, 0.0, 0.70710678118};
    assertElbowGlobalPositionGivenJointAngle(diagonal_upright_left_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_diagonal_upright_right_elbow_global_position)
{
    double diagonal_upright_right_joint_angle = -M_PI_4;
    Eigen::Vector3d expected_global_position = {0.70710678118, 0.0, 0.70710678118};
    assertElbowGlobalPositionGivenJointAngle(diagonal_upright_right_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_diagonal_downward_left_elbow_global_position)
{
    double diagonal_downward_left_joint_angle = 3.0 * M_PI_4;
    Eigen::Vector3d expected_global_position = {-0.70710678118, 0.0, -0.70710678118};
    assertElbowGlobalPositionGivenJointAngle(diagonal_downward_left_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_diagonal_downward_right_elbow_global_position)
{
    double diagonal_downward_right_joint_angle = -3.0 * M_PI_4;
    Eigen::Vector3d expected_global_position = {0.70710678118, 0.0, -0.70710678118};
    assertElbowGlobalPositionGivenJointAngle(diagonal_downward_right_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_upright_elbow_global_rotation)
{
    double upright_joint_angle = 0.0;
    Eigen::Matrix3d expected_global_rotation = Eigen::Matrix3d::Identity();
    assertElbowGlobalRotationGivenJointAngle(upright_joint_angle, expected_global_rotation);
}

TEST_F(RobotDescriptionTest, should_get_downward_elbow_global_rotation)
{
    double downward_joint_angle = M_PI;
    Eigen::Matrix3d expected_global_rotation = Eigen::Matrix3d();
    expected_global_rotation << -1.0, 0.0, 0.0,
                                0.0, 1.0,  0.0,
                                0.0, 0.0, -1.0;
    assertElbowGlobalRotationGivenJointAngle(downward_joint_angle, expected_global_rotation);
}

TEST_F(RobotDescriptionTest, should_get_left_sideways_elbow_global_rotation)
{
    double left_sideways_joint_angle = M_PI_2;
    Eigen::Matrix3d expected_global_rotation = Eigen::Matrix3d();
    expected_global_rotation << 0.0, 0.0, -1.0,
                                0.0, 1.0, 0.0,
                                1.0, 0.0, 0.0;
    assertElbowGlobalRotationGivenJointAngle(left_sideways_joint_angle, expected_global_rotation);
}

TEST_F(RobotDescriptionTest, should_get_right_sideways_elbow_global_rotation)
{
    double right_sideways_joint_angle = -M_PI_2;
    Eigen::Matrix3d expected_global_rotation = Eigen::Matrix3d();
    expected_global_rotation << 0.0, 0.0, 1.0,
                                0.0, 1.0, 0.0,
                                -1.0, 0.0, 0.0;
    assertElbowGlobalRotationGivenJointAngle(right_sideways_joint_angle, expected_global_rotation);
}

TEST_F(RobotDescriptionTest, should_get_right_sideways_elbow_global_position_jacobian)
{
    double right_sideways_joint_angle = -M_PI_2;
    Eigen::MatrixXd expected_global_position_jacobian = Eigen::MatrixXd::Zero(3, 1);
    expected_global_position_jacobian(0, 0) = 0.0;
    expected_global_position_jacobian(1, 0) = 0.0;
    expected_global_position_jacobian(2, 0) = 1.0;

    assertElbowGlobalPositionJacobianGivenJointAngle(right_sideways_joint_angle, expected_global_position_jacobian);
}

TEST_F(RobotDescriptionTest, should_get_right_sideways_link_elbow_global_rotation_jacobian)
{
    double right_sideways_joint_angle = -M_PI_2;
    Eigen::MatrixXd expected_global_rotation_jacobian = Eigen::MatrixXd::Zero(3, 1);
    expected_global_rotation_jacobian(0, 0) = 0.0;
    expected_global_rotation_jacobian(1, 0) = 0.0;
    expected_global_rotation_jacobian(2, 0) = 0.0;

    Eigen::MatrixXd actual_global_rotation_jacobian = getElbowGlobalRotationJacobianGivenJointAngle(right_sideways_joint_angle);

    ASSERT_NEAR(expected_global_rotation_jacobian(0, 0), actual_global_rotation_jacobian(0, 0), m_abs_error);
    ASSERT_NEAR(expected_global_rotation_jacobian(1, 0), actual_global_rotation_jacobian(1, 0), m_abs_error);
    ASSERT_NEAR(expected_global_rotation_jacobian(2, 0), actual_global_rotation_jacobian(2, 0), m_abs_error);
}

TEST_F(RobotDescriptionTest, should_get_right_sideways_joint_elbow_global_rotation_jacobian)
{
    double right_sideways_joint_angle = -M_PI_2;
    Eigen::MatrixXd expected_global_rotation_jacobian = Eigen::MatrixXd::Zero(3, 1);
    expected_global_rotation_jacobian(0, 0) = 0.0;
    expected_global_rotation_jacobian(1, 0) = 0.0;
    expected_global_rotation_jacobian(2, 0) = 0.0;

    Eigen::MatrixXd actual_global_rotation_jacobian = m_robot_description->findNodes({"joint_elbow"})[0]->getGlobalRotationJacobian({"joint_elbow"}, {right_sideways_joint_angle});

    ASSERT_NEAR(expected_global_rotation_jacobian(0, 0), actual_global_rotation_jacobian(0, 0), m_abs_error);
    ASSERT_NEAR(expected_global_rotation_jacobian(1, 0), actual_global_rotation_jacobian(1, 0), m_abs_error);
    ASSERT_NEAR(expected_global_rotation_jacobian(2, 0), actual_global_rotation_jacobian(2, 0), m_abs_error);
}

// Not worth testing for the expressions of endpoint, due to the inherent randomness of the ordering of the ginac expressions.
// TEST_F(RobotDescriptionTest, should_get_expected_endpoint_global_position_expression)
// {
//     std::vector<std::string> node_names = {"link_endpoint"};
//     std::string expected_global_position_expression_x = "sin(-q_{joint_shoulder})+cos(-q_{joint_shoulder})*sin(-q_{joint_elbow})+sin(-q_{joint_elbow})*cos(-q_{joint_shoulder})";
//     std::string expected_global_position_expression_y = "0.0";
//     std::string expected_global_position_expression_z = "-sin(-q_{joint_elbow})*sin(-q_{joint_shoulder})+cos(-q_{joint_elbow})*cos(-q_{joint_shoulder})+cos(-q_{joint_elbow})";
    
//     GiNaC::matrix actual_global_position_expression_matrix = m_robot_description->findNodes(node_names)[0]->getGlobalPositionExpression();
//     std::string actual_global_position_expression_x = convertGinacExpressionToString(actual_global_position_expression_matrix(0, 0));
//     std::string actual_global_position_expression_y = convertGinacExpressionToString(actual_global_position_expression_matrix(1, 0));
//     std::string actual_global_position_expression_z = convertGinacExpressionToString(actual_global_position_expression_matrix(2, 0));

//     ASSERT_EQ(expected_global_position_expression_x, actual_global_position_expression_x);
//     ASSERT_EQ(expected_global_position_expression_y, actual_global_position_expression_y);
//     ASSERT_EQ(expected_global_position_expression_z, actual_global_position_expression_z);
// }

TEST_F(RobotDescriptionTest, should_get_upright_endpoint_global_position)
{
    double upright_shoulder_joint_angle = 0.0;
    double upright_elbow_joint_angle = 0.0;
    Eigen::Vector3d expected_global_position = {0.0, 0.0, 2.0};
    assertEndpointGlobalPositionGivenJointAngles(upright_shoulder_joint_angle, upright_elbow_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_downward_endpoint_global_position)
{
    double downward_shoulder_joint_angle = M_PI;
    double downward_elbow_joint_angle = 0.0;
    Eigen::Vector3d expected_global_position = {0.0, 0.0, -2.0};
    assertEndpointGlobalPositionGivenJointAngles(downward_shoulder_joint_angle, downward_elbow_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_zero_endpoint_global_position_from_upright_elbow)
{
    double left_sideways_shoulder_joint_angle = 0.0;
    double left_sideways_elbow_joint_angle = M_PI;
    Eigen::Vector3d expected_global_position = {0.0, 0.0, 0.0};
    assertEndpointGlobalPositionGivenJointAngles(left_sideways_shoulder_joint_angle, left_sideways_elbow_joint_angle, expected_global_position);
}

TEST_F(RobotDescriptionTest, should_get_upright_endpoint_global_rotation)
{
    double upright_shoulder_joint_angle = 0.0;
    double upright_elbow_joint_angle = 0.0;
    Eigen::Matrix3d expected_global_rotation = Eigen::Matrix3d::Identity();
    assertEndpointGlobalRotationGivenJointAngles(upright_shoulder_joint_angle, upright_elbow_joint_angle, expected_global_rotation);
}

TEST_F(RobotDescriptionTest, should_get_downward_endpoint_global_rotation)
{
    double downward_shoulder_joint_angle = M_PI;
    double downward_elbow_joint_angle = M_PI;
    Eigen::Matrix3d expected_global_rotation = Eigen::Matrix3d::Identity();
    assertEndpointGlobalRotationGivenJointAngles(downward_shoulder_joint_angle, downward_elbow_joint_angle, expected_global_rotation);
}



// NOLINTEND
#endif