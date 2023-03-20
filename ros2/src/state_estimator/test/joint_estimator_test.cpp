#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// #include "mocks/mock_state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "state_estimator/state_estimator.hpp"
#include <cmath>
#include <cstdio>
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

class JointEstimatorTest : public testing::Test {
protected:
//    void SetUp() override
//    {
//        state_estimator = StateEstimator();
//        JointEstimator je = state_estimator.get_joint_estimator();
//        joint_estimator = std::make_unique<je>();
////        joint_estimator = std::make_unique<JointEstimator>(&state_estimator);
//    }
//
//    StateEstimator state_estimator;
//    std::unique_ptr<JointEstimator> joint_estimator;
};

//TEST_F(JointEstimatorTest, setupTest)
//{
////    std::vector<JointContainer> joints = joint_estimator->get_joints();
////    ASSERT_EQ(joints.size(), 0);
////
//
////    sensor_msgs::msg::JointState new_single_joint_state;
////    new_single_joint_state.position = {1.0};
////    new_single_joint_state.velocity = {1.0};
////    new_single_joint_state.effort = {1.0};
////
////
////    joint_estimator->set_joint_states(&new_single_joint_state);
//
//    //    MockStateEstimator mock_state_estimator;
//    //    // JointEstimator test_joint_estimator =
//    //    // JointEstimator(state_estimator.get(),state_estimator->get_initial_joint_states()); JointContainer
//    //    test_joint;
//    //
//    //    // tf2::Quaternion q;
//    //    // geometry_msgs::msg::Quaternion q_joint;
//    //    // q.setRPY(1,1,1);
//    //    // q.normalize();
//    //    // tf2::convert(q, test_joint.frame.transform.rotation);
//    //    // test_joint_estimator.set_individual_joint_state("right_origin", 1);    //    MockStateEstimator mock_state_estimator;
//    //    // JointEstimator test_joint_estimator =
//    //    // JointEstimator(state_estimator.get(),state_estimator->get_initial_joint_states()); JointContainer
//    //    test_joint;
//    //
//    //    // tf2::Quaternion q;
//    //    // geometry_msgs::msg::Quaternion q_joint;
//    //    // q.setRPY(1,1,1);
//    //    // q.normalize();
//    //    // tf2::convert(q, test_joint.frame.transform.rotation);
//    //    // test_joint_estimator.set_individual_joint_state("right_origin", 1);
//    //    ASSERT_EQ(1, 1);
//    //    // ASSERT_EQ(test_joint.com.mass, test_joint.com.mass);
//    //    //
//    //    ASSERT_EQ(test_joint_estimator.get_individual_joint("right_origin"),test_joint_estimator.get_individual_joint("right_origin"));
//    //    // state_estimator->m_joint_estimator.set_individual_joint_state("left_origin", 1);
//    //    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("left_origin"),1);
//    //
//    //    // state_estimator->m_joint_estimator.set_individual_joint_state("right_origin", 10);
//    //    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("right_origin"),10);
//    //    // state_estimator->m_joint_estimator.set_individual_joint_state("left_origin", 10);
//    //    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("left_origin"),10);
//    //    ASSERT_EQ(1, 1);
//    //    ASSERT_EQ(1, 1);
//    //    // ASSERT_EQ(test_joint.com.mass, test_joint.com.mass);
//    //    //
//    //    ASSERT_EQ(test_joint_estimator.get_individual_joint("right_origin"),test_joint_estimator.get_individual_joint("right_origin"));
//    //    // state_estimator->m_joint_estimator.set_individual_joint_state("left_origin", 1);
//    //    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("left_origin"),1);
//    //
//    //    // state_estimator->m_joint_estimator.set_individual_joint_state("right_origin", 10);
//    //    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("right_origin"),10);
//    //    // state_estimator->m_joint_estimator.set_individual_joint_state("left_origin", 10);
//    //    // ASSERT_EQ(state_estimator->m_joint_estimator.get_individual_joint("left_origin"),10);
//    //    ASSERT_EQ(1, 1);
//}

// NOLINTEND
#endif
