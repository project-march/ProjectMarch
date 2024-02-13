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
#include "march_state_estimator/robot_description.hpp"
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
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_torque_converter = std::make_unique<TorqueConverter>(m_robot_description);
    }

    void setupHennieWithKoen()
    {
        std::string yaml_filename = "robot_definition-hennie_with_koen.yaml";
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_torque_converter = std::make_unique<TorqueConverter>(m_robot_description);
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

// NOLINTBEGIN
#endif // __clang_analyzer__