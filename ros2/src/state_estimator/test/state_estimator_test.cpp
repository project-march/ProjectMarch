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

// NOLINTEND
#endif