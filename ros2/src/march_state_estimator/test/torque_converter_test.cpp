/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "march_state_estimator/robot_description/robot_description.hpp"
#include "march_state_estimator/torque_converter.hpp"

class TorqueConverterTest : public ::testing::Test {
public:
    TorqueConverterTest() = default;
    ~TorqueConverterTest() override = default;

protected:
    void TearDown() override
    {
        m_torque_converter.reset();
        m_robot_description.reset();
    }

    void setupRotationalTestSetup()
    {
        std::string yaml_filename = "robot_definition-rotational_test_setup.yaml";
        std::string urdf_file_path = ament_index_cpp::get_package_share_directory("march_description") + "/models/rotational_test_setup.urdf";
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_torque_converter = std::make_unique<TorqueConverter>(m_robot_description, urdf_file_path);
    }

    void setupHennieWithKoen()
    {
        std::string yaml_filename = "robot_definition-hennie_with_koen.yaml";
        std::string urdf_file_path = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf";
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_torque_converter = std::make_unique<TorqueConverter>(m_robot_description, urdf_file_path);
    }

    RobotNode::JointNameToValueMap createJointValueForRotationalTestSetup(const double& joint_value)
    {
        RobotNode::JointNameToValueMap joint_zero_values = { { "bar", joint_value } };
        return joint_zero_values;
    }

    RobotNode::JointNameToValueMap createZeroJointValueForRotationalTestSetup()
    {
        return createJointValueForRotationalTestSetup(0.0);
    }

    std::unique_ptr<TorqueConverter> m_torque_converter;
    std::shared_ptr<RobotDescription> m_robot_description;
};

TEST_F(TorqueConverterTest, test_should_able_to_setup_torque_converter_with_rotational_test_setup)
{
    setupRotationalTestSetup();
    EXPECT_TRUE(m_torque_converter != nullptr);
}

TEST_F(TorqueConverterTest, test_should_able_to_setup_torque_converter_with_hennie_with_koen)
{
    setupHennieWithKoen();
    EXPECT_TRUE(m_torque_converter != nullptr);
}

TEST_F(TorqueConverterTest, test_should_be_able_to_get_joint_names_from_rotational_test_setup)
{
    setupRotationalTestSetup();
    std::vector<std::string> expected_joint_names = { "bar" };
    std::vector<std::string> actual_joint_names = m_torque_converter->getJointNames();

    EXPECT_EQ(expected_joint_names, actual_joint_names);
}

TEST_F(TorqueConverterTest, test_should_be_able_to_get_joint_names_from_hennie_with_koen)
{
    setupHennieWithKoen();
    std::vector<std::string> expected_joint_names = { "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle",
        "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle" };
    std::vector<std::string> actual_joint_names = m_torque_converter->getJointNames();

    EXPECT_EQ(expected_joint_names, actual_joint_names);
}

TEST_F(
    TorqueConverterTest, test_should_get_same_array_size_from_dynamical_torque_expressions_from_rotational_test_setup)
{
    setupRotationalTestSetup();
    unsigned int expected_dynamical_torque_size = 1;

    unsigned int actual_dynamical_torque_size
        = m_torque_converter
              ->getDynamicalTorques(createZeroJointValueForRotationalTestSetup(),
                  createZeroJointValueForRotationalTestSetup(), createZeroJointValueForRotationalTestSetup())
              .size();

    EXPECT_EQ(expected_dynamical_torque_size, actual_dynamical_torque_size);
}

TEST_F(
    TorqueConverterTest, test_should_get_same_array_size_from_dynamical_joint_accelerations_from_rotational_test_setup)
{
    setupRotationalTestSetup();
    unsigned int expected_dynamical_joint_acceleration_size = 1;

    unsigned int actual_dynamical_joint_acceleration_size
        = m_torque_converter
              ->getDynamicalJointAccelerations(
                  createZeroJointValueForRotationalTestSetup(), createZeroJointValueForRotationalTestSetup())
              .size();

    EXPECT_EQ(expected_dynamical_joint_acceleration_size, actual_dynamical_joint_acceleration_size);
}

TEST_F(TorqueConverterTest, test_should_get_zero_dynamical_joint_acceleration_from_rotational_test_setup)
{
    setupRotationalTestSetup();
    double expected_dynamical_joint_acceleration = 0.0;

    double actual_dynamical_joint_acceleration
        = m_torque_converter
              ->getDynamicalJointAccelerations(
                  createZeroJointValueForRotationalTestSetup(), createZeroJointValueForRotationalTestSetup())
              .at("bar");

    EXPECT_EQ(expected_dynamical_joint_acceleration, actual_dynamical_joint_acceleration);
}

TEST_F(TorqueConverterTest, test_should_get_nonzero_static_dynamical_torque_from_rotational_test_setup_in_zero_position)
{
    setupRotationalTestSetup();
    RobotNode::JointNameToValueMap joint_positions = { { "bar", 0.0 } };
    RobotNode::JointNameToValueMap joint_velocities = { { "bar", 0.0 } };
    RobotNode::JointNameToValueMap joint_accelerations = { { "bar", 0.0 } };

    double actual_dynamical_torque_at_joint_bar
        = m_torque_converter
              ->getDynamicalTorques(joint_positions, joint_velocities, joint_accelerations).at("bar");

    std::cout << "actual_dynamical_torque_at_joint_bar: " << actual_dynamical_torque_at_joint_bar << std::endl;

    EXPECT_NE(0.0, actual_dynamical_torque_at_joint_bar);
}

TEST_F(TorqueConverterTest, test_should_get_zero_static_dynamical_torque_from_rotational_test_setup_in_upright_position)
{
    setupRotationalTestSetup();
    RobotNode::JointNameToValueMap joint_positions = { { "bar", M_PI_4 } };
    RobotNode::JointNameToValueMap joint_velocities = { { "bar", 0.0 } };
    RobotNode::JointNameToValueMap joint_accelerations = { { "bar", 0.0 } };

    double actual_dynamical_torque_at_joint_bar
        = m_torque_converter
              ->getDynamicalTorques(joint_positions, joint_velocities, joint_accelerations).at("bar");

    std::cout << "actual_dynamical_torque_at_joint_bar: " << actual_dynamical_torque_at_joint_bar << std::endl;

    EXPECT_NEAR(0.0, actual_dynamical_torque_at_joint_bar, 1e-12);
}

TEST_F(TorqueConverterTest, test_should_get_zero_static_dynamical_torque_from_rotational_test_setup_in_hanging_position)
{
    setupRotationalTestSetup();
    RobotNode::JointNameToValueMap joint_positions = { { "bar", -3 * M_PI_4 } };
    RobotNode::JointNameToValueMap joint_velocities = { { "bar", 0.0 } };
    RobotNode::JointNameToValueMap joint_accelerations = { { "bar", 0.0 } };

    double actual_dynamical_torque_at_joint_bar
        = m_torque_converter
              ->getDynamicalTorques(joint_positions, joint_velocities, joint_accelerations).at("bar");

    std::cout << "actual_dynamical_torque_at_joint_bar: " << actual_dynamical_torque_at_joint_bar << std::endl;

    EXPECT_NEAR(0.0, actual_dynamical_torque_at_joint_bar, 1e-12);
}

TEST_F(TorqueConverterTest, test_should_get_max_static_dynamical_torque_from_rotational_test_setup_in_horizontal_position)
{
    setupRotationalTestSetup();
    RobotNode::JointNameToValueMap joint_positions = { { "bar", -M_PI_4 } };
    RobotNode::JointNameToValueMap joint_velocities = { { "bar", 0.0 } };
    RobotNode::JointNameToValueMap joint_accelerations = { { "bar", 0.0 } };

    double actual_dynamical_torque_at_joint_bar
        = m_torque_converter
              ->getDynamicalTorques(joint_positions, joint_velocities, joint_accelerations).at("bar");

    std::cout << "actual_dynamical_torque_at_joint_bar: " << actual_dynamical_torque_at_joint_bar << std::endl;

    EXPECT_NEAR(-2.3383617890432653, actual_dynamical_torque_at_joint_bar, 1e-12);
}

TEST_F(TorqueConverterTest, test_should_be_able_to_calculate_external_forces_from_dummy_total_joint_torques_and_dummy_dynamical_joint_torques_from_rotational_test_setup)
{
    setupRotationalTestSetup();
    RobotNode::JointNameToValueMap joint_total_torques = { { "bar", 3.0 } };
    RobotNode::JointNameToValueMap joint_dynamical_torques = { { "bar", 2.0 } };
    RobotNode::JointNameToValueMap expected_external_torques = { { "bar", 1.0 } };

    RobotNode::JointNameToValueMap actual_external_torques
        = m_torque_converter->getExternalTorques(joint_total_torques, joint_dynamical_torques);

    EXPECT_EQ(expected_external_torques, actual_external_torques);
}

TEST_F(TorqueConverterTest, test_should_be_able_to_calculate_external_force_in_upright_position_with_dummy_external_torque_from_rotational_test_setup)
{
    setupRotationalTestSetup();
    RobotNode::JointNameToValueMap joint_positions = { { "bar", M_PI_4 } };
    RobotNode::JointNameToValueMap external_torques = { { "bar", 1.0 } };
    Eigen::Vector3d expected_external_force = { 2.9673067882, 0.0, 9.09709e-17};

    Eigen::Vector3d actual_external_force
        = m_torque_converter->getExternalForceByNode("weight", joint_positions, external_torques);

    std::cout << "Expected external force: "<< std::endl << expected_external_force << std::endl;
    std::cout << "Actual external force: "<< std::endl << actual_external_force << std::endl;
    std::cout << "Difference: "<< std::endl << expected_external_force - actual_external_force << std::endl;
    ASSERT_TRUE(expected_external_force.isApprox(actual_external_force, 1e-3));
}

TEST_F(TorqueConverterTest, test_should_be_able_to_calculate_external_force_in_upright_position_with_zero_external_torque_from_rotational_test_setup)
{
    setupRotationalTestSetup();
    RobotNode::JointNameToValueMap joint_positions = { { "bar", M_PI_4 } };
    RobotNode::JointNameToValueMap external_torques = { { "bar", 0.0 } };
    Eigen::Vector3d expected_external_force = { 0.0, 0.0, 0.0};

    Eigen::Vector3d actual_external_force
        = m_torque_converter->getExternalForceByNode("weight", joint_positions, external_torques);

    std::cout << "Expected external force: "<< std::endl << expected_external_force << std::endl;
    std::cout << "Actual external force: "<< std::endl << actual_external_force << std::endl;
    std::cout << "Difference: "<< std::endl << expected_external_force - actual_external_force << std::endl;
    ASSERT_TRUE(expected_external_force.isApprox(actual_external_force));
}

TEST_F(TorqueConverterTest, test_should_be_able_to_calculate_external_force_in_left_foot_in_upright_position_with_zero_external_torque_from_hennie_with_koen)
{
    setupHennieWithKoen();
    RobotNode::JointNameToValueMap joint_positions = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 },
        { "left_ankle", 0.0 }, { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };
    RobotNode::JointNameToValueMap external_torques = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 },
        { "left_ankle", 0.0 }, { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };
    Eigen::Vector3d expected_left_foot_external_force = { 0.0, 0.0, 0.0};

    Eigen::Vector3d actual_left_foot_external_force
        = m_torque_converter->getExternalForceByNode("L_foot", joint_positions, external_torques);

    std::cout << "Expected external force: "<< std::endl << expected_left_foot_external_force << std::endl;
    std::cout << "Actual external force: "<< std::endl << actual_left_foot_external_force << std::endl;
    std::cout << "Difference: "<< std::endl << expected_left_foot_external_force - actual_left_foot_external_force << std::endl;
    ASSERT_TRUE(expected_left_foot_external_force.isApprox(actual_left_foot_external_force));
}

TEST_F(TorqueConverterTest, test_should_be_able_to_calculate_external_force_in_left_foot_in_upright_position_with_dummy_external_torque_from_hennie_with_koen)
{
    setupHennieWithKoen();
    RobotNode::JointNameToValueMap joint_positions = { { "left_hip_aa", 0.0 }, { "left_hip_fe", 0.0 }, { "left_knee", 0.0 },
        { "left_ankle", 0.0 }, { "right_hip_aa", 0.0 }, { "right_hip_fe", 0.0 }, { "right_knee", 0.0 }, { "right_ankle", 0.0 } };
    RobotNode::JointNameToValueMap external_torques = { { "left_hip_aa", 1.0 }, { "left_hip_fe", 2.0 }, { "left_knee", 3.0 },
        { "left_ankle", 4.0 }, { "right_hip_aa", 5.0 }, { "right_hip_fe", 6.0 }, { "right_knee", 7.0 }, { "right_ankle", 8.0 } };
    Eigen::Vector3d expected_left_foot_external_force = { 2.69125, 11.4669, 379.401 };

    Eigen::Vector3d actual_left_foot_external_force
        = m_torque_converter->getExternalForceByNode("L_foot", joint_positions, external_torques);

    std::cout << "Expected external force: "<< std::endl << expected_left_foot_external_force << std::endl;
    std::cout << "Actual external force: "<< std::endl << actual_left_foot_external_force << std::endl;
    std::cout << "Difference: "<< std::endl << expected_left_foot_external_force - actual_left_foot_external_force << std::endl;
    ASSERT_TRUE(expected_left_foot_external_force.isApprox(actual_left_foot_external_force, 1e-3));
}

// NOLINTBEGIN
#endif // __clang_analyzer__