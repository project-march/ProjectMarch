//
// Created by Marco Bak on 22-2-23.
//
#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.

#include "sensor_msgs/msg/joint_state.hpp"
#include "state_estimator.hpp"
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class StateEstimatorTest : public testing::Test {
protected:
    void SetUp() override
    {
        state_estimator = std::make_unique<StateEstimator>();
    }
    std::unique_ptr<StateEstimator> state_estimator;
};

TEST_F(StateEstimatorTest, UpdateSensorPressureTest)
{
    std::vector<std::string> names = { "test", "test1" };
    std::vector<double> pressure_values = { 1, 2 };
    std::map<std::string, double> expected = { { "test", 1 }, { "test1", 2 } };
    ASSERT_EQ(expected, state_estimator->update_pressure_sensors_data(names, pressure_values));
}

// NOLINTEND
#endif