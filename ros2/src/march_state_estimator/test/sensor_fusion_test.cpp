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
#include "geometry_msgs/msg/quaternion.hpp"
#include "march_state_estimator/sensor_fusion.hpp"

class SensorFusionTest : public ::testing::Test {
public:
    SensorFusionTest() = default;
    ~SensorFusionTest() override = default;

protected:
    void TearDown() override
    {
        m_sensor_fusion.reset();
        m_robot_description.reset();
    }

    void setupRotationalTestSetup()
    {
        std::string yaml_filename = "robot_definition-rotational_test_setup.yaml";
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_sensor_fusion = std::make_unique<SensorFusion>(m_robot_description);

        std::vector<std::string> joint_names = { "bar" };
        m_sensor_fusion->configureJointNames(joint_names);
    }

    void setupHennieWithKoen()
    {
        std::string yaml_filename = "robot_definition-hennie_with_koen.yaml";
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_sensor_fusion = std::make_unique<SensorFusion>(m_robot_description);
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

    std::shared_ptr<RobotDescription> m_robot_description;
    std::unique_ptr<SensorFusion> m_sensor_fusion;
};

TEST_F(SensorFusionTest, test_should_create_sensor_fusion_instance_and_configure_for_rotational_test_setup)
{
    ASSERT_NO_FATAL_FAILURE();
}

TEST_F(SensorFusionTest, test_should_update_joint_state_for_rotational_test_setup)
{
    setupRotationalTestSetup();
    sensor_msgs::msg::JointState::SharedPtr joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state->name.push_back("bar");
    joint_state->position.push_back(0.0);
    joint_state->velocity.push_back(0.0);
    joint_state->effort.push_back(0.0);
    m_sensor_fusion->updateJointState(joint_state);
    ASSERT_NO_FATAL_FAILURE();
}

// NOLINTEND
#endif // __clang_analyzer__