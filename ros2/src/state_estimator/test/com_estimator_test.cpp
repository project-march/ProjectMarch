//
// Created by marco on 14-2-23.
//
#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.

#include "com_estimator.hpp"
#include "mocks/mock_state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class ComEstimatorTest : public testing::Test {
protected:
    void SetUp() override
    {
        com_estimator = std::make_unique<ComEstimator>();
    }
    std::unique_ptr<ComEstimator> com_estimator;
};

TEST_F(ComEstimatorTest, setComTest)
{
    std::vector<CenterOfMass> masses;
    CenterOfMass com;
    com.mass = 1;
    com.position.point.x = 0;
    com.position.point.y = 0;
    com.position.point.z = 0;
    masses.push_back(com);
    com_estimator->set_com_state(masses);
    ASSERT_EQ(com_estimator->get_com_state(), com);
}

// NOLINTEND
#endif
