#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
// #include "ik_solver.cpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "torque_converter/torque_converter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

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

#endif
