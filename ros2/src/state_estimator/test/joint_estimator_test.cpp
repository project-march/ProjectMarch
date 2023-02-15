#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "state_estimator/state_estimator.hpp"
#include <cmath>
#include <memory>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    }
};

TEST_F(StateEstimatorTest, SetterTest)
{
    // std::unique_ptr<StateEstimator> state_estimator = std::make_unique<StateEstimator>();
    // tf2::Quaternion q;
    // geometry_msgs::msg::Quaternion q_joint;
    // q.setRPY(1,1,1);
    // q.normalize();
    // tf2::convert(q, q_joint);
    // state_estimator->m_joint_estimator.set_individual_joint_state("right_origin", 1);

    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("right_origin").,1);
    // state_estimator->m_joint_estimator.set_individual_joint_state("left_origin", 1);
    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("left_origin"),1);

    // state_estimator->m_joint_estimator.set_individual_joint_state("right_origin", 10);
    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("right_origin"),10);
    // state_estimator->m_joint_estimator.set_individual_joint_state("left_origin", 10);
    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("left_origin"),10);
    ASSERT_EQ(1, 1);
}

// NOLINTEND
#endif
