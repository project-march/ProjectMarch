#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
// #include "ik_solver.cpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "torque_converter/torque_converter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>
#include <utility>
#include <Eigen/Core>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;
using namespace Eigen;

class TorqueConverterTest : public testing::Test {
protected:
    void SetUp() override
    {
        this->torque_converter = std::make_unique<TorqueConverter>();
    }

    std::unique_ptr<TorqueConverter> torque_converter;
};

TEST_F(TorqueConverterTest, mockTest)
{
    ASSERT_EQ(1, 1);
}

TEST_F(TorqueConverterTest, vel_test)
{
    VectorXd des_pos(5);
    des_pos << 4.0, 2.0, 9.0, 5.0, 6.0;
    VectorXd correct_vel(4);
    correct_vel << -2.0, 7.0, -4.0, 1.0;

    TorqueConverter myTC;
    myTC.set_desired_pos(des_pos);
    VectorXd vel_des = myTC.get_desired_vel();
    ASSERT_EQ(vel_des, correct_vel);
}
#endif
