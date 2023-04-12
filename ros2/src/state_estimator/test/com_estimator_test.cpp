//
// Created by marco on 14-2-23.
//
#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.

#include "com_estimator.hpp"
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

TEST_F(ComEstimatorTest, setEmptyComTest)
{
    std::vector<CenterOfMass> masses;
    CenterOfMass com;
    com_estimator->set_com_state(masses);
    ASSERT_TRUE(std::isnan(com_estimator->get_com_state().position.point.x));
    ASSERT_TRUE(std::isnan(com_estimator->get_com_state().position.point.y));
    ASSERT_TRUE(std::isnan(com_estimator->get_com_state().position.point.z));
    ASSERT_EQ(com_estimator->get_com_state().mass, 0);
}

TEST_F(ComEstimatorTest, setMultipleComTest)
{
    std::vector<CenterOfMass> masses;
    CenterOfMass com1;
    com1.mass = 2;
    com1.position.point.x = 1;
    com1.position.point.y = 2;
    com1.position.point.z = 1;
    masses.push_back(com1);
    CenterOfMass com2;
    com2.mass = 2;
    com2.position.point.x = 2;
    com2.position.point.y = 2;
    com2.position.point.z = 1;
    masses.push_back(com2);
    CenterOfMass com3;
    com3.mass = 2;
    com3.position.point.x = 1;
    com3.position.point.y = 1;
    com3.position.point.z = 1;
    masses.push_back(com3);
    CenterOfMass total;
    total.mass = com1.mass + com2.mass + com3.mass;
    total.position.point.x = (com1.position.point.x*com1.mass+com2.position.point.x*com2.mass+com3.position.point.x*com3.mass)/(com1.mass+com2.mass+com3.mass);
    total.position.point.y = (com1.position.point.y*com1.mass+com2.position.point.y*com2.mass+com3.position.point.y*com3.mass)/(com1.mass+com2.mass+com3.mass);
    total.position.point.z = (com1.position.point.z*com1.mass+com2.position.point.z*com2.mass+com3.position.point.z*com3.mass)/(com1.mass+com2.mass+com3.mass);
    com_estimator->set_com_state(masses);
    ASSERT_EQ(com_estimator->get_com_state(), total);
}

// NOLINTEND
#endif
