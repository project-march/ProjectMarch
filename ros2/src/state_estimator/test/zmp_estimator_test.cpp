//
// Created by rixt on 20-3-23.
//

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.

#include "zmp_estimator.hpp"
#include "com_estimator.hpp"

//#include "mocks/mock_state_estimator.hpp"
//#include "sensor_msgs/msg/joint_state.hpp"
//#include "state_estimator.hpp"
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class ZmpEstimatorTest : public testing::Test {
protected:
    void SetUp() override
    {
        zmp_estimator = std::make_unique<ZmpEstimator>();

    }
    void createZMP() {
        com.mass = 1;
        com.position.point.x = 0;
        com.position.point.y = 0;
        com.position.point.z = 0;
    }
    CenterOfMass com;
    std::unique_ptr<ZmpEstimator> zmp_estimator;
};

TEST_F(ZmpEstimatorTest, setComStatesTest)
{

    ASSERT_EQ(zmp_estimator->get_com_state(), com);
}

// NOLINTEND
#endif
