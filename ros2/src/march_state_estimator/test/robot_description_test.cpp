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
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "march_state_estimator/robot_description.hpp"
#include "march_state_estimator/robot_node.hpp"
#include "march_state_estimator/robot_zmp.hpp"

class RobotDescriptionTest : public ::testing::Test {
public:
    RobotDescriptionTest() = default;
    ~RobotDescriptionTest() override = default;

protected:
    void SetUp() override
    {
        std::string yaml_filename = "robot_definition-hennie_with_koen.yaml";
        m_robot_description = std::make_unique<RobotDescription>(yaml_filename);
    }

    void TearDown() override
    {
        this->m_robot_description.reset();
    }

    std::unique_ptr<RobotDescription> m_robot_description;
    double m_abs_error = 1e-3;

    const unsigned int NORM_ORDER = 2;
    const uint8_t STANCE_LEG_BOTH = 0b11;
    const uint8_t STANCE_LEG_LEFT = 0b01;
    const uint8_t STANCE_LEG_RIGHT = 0b10;
    const unsigned int NUM_POLYGONS_AXES = 2;
    const unsigned int NUM_POLYGON_VERTICES = 4;
    const unsigned int NUM_COLS = 3;
};

TEST_F(RobotDescriptionTest, test_should_create_robot_description_instance_and_configure)
{
    ASSERT_NO_FATAL_FAILURE();
}

TEST_F(RobotDescriptionTest, test_should_able_to_evaluate_linear_position_in_a_robot_node)
{
    RobotNode::JointNameToValueMap empty_joint_values
        = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 }, { "left_ankle", 0.0 },
              { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };
    
    ASSERT_NO_FATAL_FAILURE(m_robot_description->findNode("L_foot")->getGlobalPosition(empty_joint_values));
}

TEST_F(RobotDescriptionTest, test_should_able_to_evaluate_linear_velocity_in_a_robot_node)
{
    RobotNode::JointNameToValueMap empty_joint_values
        = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 }, { "left_ankle", 0.0 },
              { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };
    
    ASSERT_NO_FATAL_FAILURE(m_robot_description->findNode("L_foot")->getGlobalVelocity(empty_joint_values, empty_joint_values));
}

TEST_F(RobotDescriptionTest, test_should_able_to_evaluate_linear_acceleration_in_a_robot_node)
{
    RobotNode::JointNameToValueMap empty_joint_values
        = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 }, { "left_ankle", 0.0 },
              { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };
    
    ASSERT_NO_FATAL_FAILURE(m_robot_description->findNode("L_foot")->getGlobalAcceleration(empty_joint_values, empty_joint_values, empty_joint_values));
}

TEST_F(RobotDescriptionTest, test_should_able_to_find_and_store_pointer_to_robot_zmp)
{
    std::shared_ptr<RobotNode> zmp_node = m_robot_description->findNode("zmp");
    ASSERT_NE(zmp_node, nullptr);
}

TEST_F(RobotDescriptionTest, test_should_able_to_get_foot_bounding_polygon)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    Eigen::MatrixXd expected_foot_bounding_polygon(NUM_POLYGON_VERTICES, NUM_POLYGONS_AXES);
    expected_foot_bounding_polygon << 1.15, 2.0675, 0.85, 2.0675, 0.85, 1.9325, 1.15, 1.9325;

    Eigen::MatrixXd actual_foot_bounding_polygon = zmp_node->getFootBoundingPolygon(Eigen::Vector2d(1.0, 2.0));

    ASSERT_EQ(actual_foot_bounding_polygon.rows(), NUM_POLYGON_VERTICES);
    ASSERT_EQ(actual_foot_bounding_polygon.cols(), NUM_POLYGONS_AXES);
    ASSERT_TRUE(actual_foot_bounding_polygon.isApprox(expected_foot_bounding_polygon, m_abs_error));
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_compute_polygon_union_coefficients)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    Eigen::MatrixXd polygon1(NUM_POLYGON_VERTICES, NUM_POLYGONS_AXES);
    polygon1 << -1.0, 2.0, -2.0, 2.0, -2.0, 1.0, -1.0, 1.0;
    Eigen::MatrixXd polygon2(NUM_POLYGON_VERTICES, NUM_POLYGONS_AXES);
    polygon2 << 1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0;
    Eigen::MatrixXd expected_union_polygon_coefficients(NUM_POLYGONS_AXES, NUM_COLS);
    expected_union_polygon_coefficients << -2.0, 1.0, -0.5, 0.0, 2.0, 1.0;

    Eigen::MatrixXd actual_union_polygon_coefficients = zmp_node->computePolygonUnionCoefficients(polygon1, polygon2);

    ASSERT_EQ(actual_union_polygon_coefficients.rows(), NUM_POLYGONS_AXES);
    ASSERT_EQ(actual_union_polygon_coefficients.cols(), NUM_COLS);
    ASSERT_TRUE(actual_union_polygon_coefficients.isApprox(expected_union_polygon_coefficients, m_abs_error));
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_compute_polygon_on_two_stance_legs)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setFootPositions(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, -1.0, 0.0));
    zmp_node->setStanceLeg(STANCE_LEG_BOTH);
    Eigen::MatrixXd expected_union_polygon_coefficients(NUM_POLYGONS_AXES, NUM_COLS);
    expected_union_polygon_coefficients << -0.15, 0.15, 0.0, -1.0675, 1.0675, 0.0;

    Eigen::MatrixXd actual_union_polygon_coefficients = zmp_node->getFootBoundingPolygonUnionCoefficients();

    ASSERT_EQ(actual_union_polygon_coefficients.rows(), NUM_POLYGONS_AXES);
    ASSERT_EQ(actual_union_polygon_coefficients.cols(), NUM_COLS);
    ASSERT_TRUE(actual_union_polygon_coefficients.isApprox(expected_union_polygon_coefficients, m_abs_error));
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_compute_polygon_on_left_stance_leg)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setFootPositions(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, -1.0, 0.0));
    zmp_node->setStanceLeg(STANCE_LEG_LEFT);
    Eigen::MatrixXd expected_union_polygon_coefficients(NUM_POLYGONS_AXES, NUM_COLS);
    expected_union_polygon_coefficients << -0.15, 0.15, 0.0, 0.9325, 1.0675, 1.0;

    Eigen::MatrixXd actual_union_polygon_coefficients = zmp_node->getFootBoundingPolygonUnionCoefficients();

    ASSERT_EQ(actual_union_polygon_coefficients.rows(), NUM_POLYGONS_AXES);
    ASSERT_EQ(actual_union_polygon_coefficients.cols(), NUM_COLS);
    ASSERT_TRUE(actual_union_polygon_coefficients.isApprox(expected_union_polygon_coefficients, m_abs_error));
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_compute_polygon_on_right_stance_leg)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setFootPositions(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, -1.0, 0.0));
    zmp_node->setStanceLeg(STANCE_LEG_RIGHT);
    Eigen::MatrixXd expected_union_polygon_coefficients(NUM_POLYGONS_AXES, NUM_COLS);
    expected_union_polygon_coefficients << -0.15, 0.15, 0.0, -1.0675, -0.9325, -1.0;

    Eigen::MatrixXd actual_union_polygon_coefficients = zmp_node->getFootBoundingPolygonUnionCoefficients();

    ASSERT_EQ(actual_union_polygon_coefficients.rows(), NUM_POLYGONS_AXES);
    ASSERT_EQ(actual_union_polygon_coefficients.cols(), NUM_COLS);
    ASSERT_TRUE(actual_union_polygon_coefficients.isApprox(expected_union_polygon_coefficients, m_abs_error));
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_normalize_x_to_support_polygon_as_zero)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setNormOrder(NORM_ORDER);
    double expected_normalized_position = 0.0;

    double actual_normalized_position = zmp_node->normalizeZmpByAxis(1.5, Eigen::Vector3d(1.0, 2.0, 1.5));

    ASSERT_EQ(actual_normalized_position, expected_normalized_position);
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_normalize_y_to_support_polygon_as_nonzero)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setNormOrder(NORM_ORDER);
    double expected_normalized_position = 1.0;

    double actual_normalized_position = zmp_node->normalizeZmpByAxis(2.0, Eigen::Vector3d(1.0, 2.0, 1.5));

    ASSERT_EQ(actual_normalized_position, expected_normalized_position);
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_get_zero_normalize_foot_positions_to_support_polygon)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setNormOrder(NORM_ORDER);
    zmp_node->setFootPositions(Eigen::Vector3d(0.0, 0.1, 0.0), Eigen::Vector3d(0.0, -0.1, 0.0));
    zmp_node->setStanceLeg(STANCE_LEG_BOTH);

    Eigen::Vector3d expected_normalized_position(0.0, 0.0, 0.0);
    Eigen::Vector3d actual_normalized_position = zmp_node->normalizeZmp(Eigen::Vector3d(0.0, 0.0, 0.0));

    ASSERT_TRUE(actual_normalized_position.isApprox(expected_normalized_position, m_abs_error));
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_get_get_zmp_global_position)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setInertialOrientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    zmp_node->setNormOrder(NORM_ORDER);
    zmp_node->setFootPositions(Eigen::Vector3d(0.3, 0.16, 0.0), Eigen::Vector3d(0.3, -0.16, 0.0));
    zmp_node->setStanceLeg(STANCE_LEG_BOTH);
    Eigen::Vector3d expected_global_position(0.0, 0.0, 0.0);
    RobotNode::JointNameToValueMap joint_positions
        = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 }, { "left_ankle", 0.0 },
              { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };

    Eigen::Vector3d actual_global_position = zmp_node->getGlobalPosition(joint_positions);

    ASSERT_NEAR(expected_global_position.x(), actual_global_position.x(), m_abs_error);
    ASSERT_NEAR(expected_global_position.y(), actual_global_position.y(), m_abs_error);
}

TEST_F(RobotDescriptionTest, test_should_be_able_to_get_zmp_global_position_jacobian_only_with_x_and_y_components)
{
    std::shared_ptr<RobotZMP> zmp_node = std::dynamic_pointer_cast<RobotZMP>(m_robot_description->findNode("zmp"));
    zmp_node->setInertialOrientation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    RobotNode::JointNameToValueMap joint_positions
        = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 }, { "left_ankle", 0.0 },
              { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };

    Eigen::MatrixXd actual_global_position_jacobian = zmp_node->getGlobalPositionJacobian(joint_positions);

    ASSERT_EQ(actual_global_position_jacobian.rows(), NUM_POLYGONS_AXES);
}

// NOLINTEND
#endif