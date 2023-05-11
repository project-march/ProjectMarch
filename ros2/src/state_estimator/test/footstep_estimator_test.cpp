#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.

#include "footstep_estimator.hpp"
#include "mocks/mock_state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "state_estimator.hpp"
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class FootstepEstimatorTest : public testing::Test {
protected:
    void SetUp() override
    {
        footstep_estimator = std::make_unique<FootstepEstimator>();
    }
    std::unique_ptr<FootstepEstimator> footstep_estimator;
};

TEST_F(FootstepEstimatorTest, feetOnGroundTest)
{
    std::vector<PressureSensor*>* sensors;

    const char prefixR = 'r';
    PressureSensor* mock_sensor_r;
    mock_sensor_r->name = "r";
    mock_sensor_r->pressure = 20;
    sensors->push_back(mock_sensor_r);

    const char prefixL = 'l';
    PressureSensor* mock_sensor_l;
    mock_sensor_l->name = "l";
    mock_sensor_l->pressure = 20;
    sensors->push_back(mock_sensor_l);

    footstep_estimator->update_feet(sensors);
    ASSERT_TRUE(footstep_estimator->get_foot_on_ground(&prefixR));
    ASSERT_TRUE(footstep_estimator->get_foot_on_ground(&prefixL));
}

TEST_F(FootstepEstimatorTest, feetOffGroundTest)
{
    std::vector<PressureSensor*>* sensors;

    const char prefixR = 'r_';
    PressureSensor* mock_sensor_r;
    mock_sensor_r->name = "r_";
    mock_sensor_r->pressure = 20;
    sensors->push_back(mock_sensor_r);

    const char prefixL = 'l_';
    PressureSensor* mock_sensor_l;
    mock_sensor_l->name = "l_";
    mock_sensor_l->pressure = 2;
    sensors->push_back(mock_sensor_l);

    footstep_estimator->update_feet(sensors);
    ASSERT_FALSE(footstep_estimator->get_foot_on_ground(&prefixR));
    ASSERT_FALSE(footstep_estimator->get_foot_on_ground(&prefixL));
}

TEST_F(FootstepEstimatorTest, updateFaultySensorTest)
{
    std::vector<PressureSensor*>* sensors;

    const char prefixR = 'r';
    PressureSensor* mock_sensor_faulty;
    mock_sensor_faulty->name = "faulty";
    mock_sensor_faulty->pressure = 20;
    sensors->push_back(mock_sensor_faulty);

    footstep_estimator->update_feet(sensors);
    ASSERT_FALSE(footstep_estimator->get_foot_on_ground(&prefixR));
}

TEST_F(FootstepEstimatorTest, unknownFootTest)
{
    std::vector<PressureSensor*>* sensors;
    const char prefixU = 'q';
    footstep_estimator->update_feet(sensors);
    ASSERT_EQ(footstep_estimator->get_foot_on_ground(&prefixU), 0);
}

TEST_F(FootstepEstimatorTest, getFeetPositionTest)
{
    geometry_msgs::msg::PointStamped right_position;
    geometry_msgs::msg::PointStamped left_position;

    right_position.point.x = 0;
    right_position.point.y = 0;
    right_position.point.z = 0;
    left_position.point.x = 1;
    left_position.point.y = 1;
    left_position.point.z = 1;

    const char prefixR = 'r';
    const char prefixL = 'l';
    Foot* rightF = footstep_estimator->get_foot(&prefixR);
    Foot* leftF = footstep_estimator->get_foot(&prefixL);
    rightF->position = right_position;
    leftF->position = left_position;

    ASSERT_EQ(footstep_estimator->get_foot_position(&prefixR).position.x, 0);
    ASSERT_EQ(footstep_estimator->get_foot_position(&prefixR).position.y, 0);
    ASSERT_EQ(footstep_estimator->get_foot_position(&prefixR).position.z, 0);
    ASSERT_EQ(footstep_estimator->get_foot_position(&prefixL).position.x, 1);
    ASSERT_EQ(footstep_estimator->get_foot_position(&prefixL).position.y, 1);
    ASSERT_EQ(footstep_estimator->get_foot_position(&prefixL).position.z, 1);
}

TEST_F(FootstepEstimatorTest, setFootSizeTest)
{
    const char prefixR = 'r';
    double width = 2.0;
    double height = 3.0;

    footstep_estimator->set_foot_size(width, height, &prefixR);
    ASSERT_EQ(footstep_estimator->get_foot(&prefixR)->width, width);
    ASSERT_EQ(footstep_estimator->get_foot(&prefixR)->height, height);
}

// NOLINTEND
#endif
