#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
#include "state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <memory>
#include <utility>

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
        this->footstep_gen = std::make_unique<StateEstimator>();
    }

    std::unique_ptr<FootstepGenerator> footstep_gen;
};

TEST_F(StateEstimatorTest, gettersTest)
{
    ASSERT_EQ(1, 8);
}

// NOLINTEND
#endif
