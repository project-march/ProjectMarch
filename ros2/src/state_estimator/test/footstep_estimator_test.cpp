//
// Created by marco on 14-2-23.
//
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

TEST_F(FootstepEstimator, feetOnGroundTest)
{
    const char prefixR = 'r';
    const char prefixL = 'l';
    std::vector<PressureSensor> sensors;
    PressureSensor mock_sensor;
    mock_sensor.name = "mock_sensor";
    CenterOfPressure cop;
    cop.pressure = 10;
    mock_sensor.centre_of_pressure = cop;
    sensors.push_back(mock_sensor);
    footstep_estimator->update_feet(sensors);
    ASSERT_TRUE(footstep_estimator->get_foot_on_ground(&prefixR));
    ASSERT_TRUE(footstep_estimator->get_foot_on_ground(&prefixL));
}

TEST_F(FootstepEstimator, feetOffGroundTest)
{
    const char prefixR = 'r';
    const char prefixL = 'l';
    std::vector<PressureSensor> sensors;
    PressureSensor mock_sensor;
    mock_sensor.name = "mock_sensor";
    CenterOfPressure cop;
    cop.pressure = 7;
    mock_sensor.centre_of_pressure = cop;
    sensors.push_back(mock_sensor);
    footstep_estimator->update_feet(sensors);
    ASSERT_FALSE(footstep_estimator->get_foot_on_ground(&prefixR));
    ASSERT_FALSE(footstep_estimator->get_foot_on_ground(&prefixL));
}

// NOLINTEND
#endif
