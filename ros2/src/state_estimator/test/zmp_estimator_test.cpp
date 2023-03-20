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
    std::unique_ptr<ZmpEstimator> zmp_estimator;
};

TEST_F(ZmpEstimatorTest, setZMPTest)
{
    zmp_estimator->set_zmp();

    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.x));
    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.y));
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);
}

TEST_F(ZmpEstimatorTest, iterateZMPTest)
{
    zmp_estimator->set_zmp();

    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.x));
    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.y));
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);

    CenterOfMass com;
    com.mass = 2;
    com.position.point.x = 1;
    com.position.point.y = 1;
    com.position.point.z = 0;

    zmp_estimator->set_com_states(com, rclcpp::Clock(RCL_ROS_TIME).now());
    zmp_estimator->set_zmp();

    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.x));
    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.y));
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);

    zmp_estimator->set_com_states(com, rclcpp::Clock(RCL_ROS_TIME).now());
    zmp_estimator->set_zmp();

    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.x));
    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.y));
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);

    zmp_estimator->set_com_states(com, rclcpp::Clock(RCL_ROS_TIME).now());
    zmp_estimator->set_zmp();

    ASSERT_EQ(zmp_estimator->get_zmp().point.x, com.position.point.x);
    ASSERT_EQ(zmp_estimator->get_zmp().point.y, com.position.point.y);
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);
}

TEST_F(ZmpEstimatorTest, accelerateZmpTest)
{
    zmp_estimator->set_zmp();

    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.x));
    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.y));
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);

    CenterOfMass com;
    com.position.point.x = 0;
    com.position.point.y = 0;
    com.position.point.z = 0;
    double seconds = 0.0;
    rclcpp::Time t0(static_cast<uint64_t>(seconds * 1e9));

    zmp_estimator->set_com_states(com, t0);
    zmp_estimator->set_zmp();

    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.x));
    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.y));
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);

    com.position.point.x = 1;
    com.position.point.y = 1;
    com.position.point.z = 1;
    seconds++;
    rclcpp::Time t1(static_cast<uint64_t>(seconds * 1e9));

    zmp_estimator->set_com_states(com, t1);
    zmp_estimator->set_zmp();

    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.x));
    ASSERT_TRUE(std::isnan(zmp_estimator->get_zmp().point.y));
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);

    com.position.point.x = 3;
    com.position.point.y = 3;
    com.position.point.z = 3;
    seconds++;
    rclcpp::Time t2(static_cast<uint64_t>(seconds * 1e9));

    zmp_estimator->set_com_states(com, t2);
    zmp_estimator->set_zmp();

    /*
     * pos [0]: 3, [1]: 1, [2]: 0
     * vel [0]: 3-1=2, [1]: 1-0=1
     * acc 2-1=1
     * 3 - 1/g * 1 = 2.898
     */

    ASSERT_TRUE(abs(zmp_estimator->get_zmp().point.x - 2.898) < 0.001);
    ASSERT_TRUE(abs(zmp_estimator->get_zmp().point.y - 2.898) < 0.001);
    ASSERT_EQ(zmp_estimator->get_zmp().point.z, 0);
}


// NOLINTEND
#endif
