#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.
// #include "ik_solver.cpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "ik_solver_buffer/ik_solver_buffer.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
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

class IkBufferTest : public testing::Test {
protected:
    void SetUp() override
    {
        this->ik_buffer = std::make_unique<BufferNode>();
    }

    std::unique_ptr<BufferNode> ik_buffer;
};

TEST_F(IkBufferTest, checkForReadyTest)
{
    geometry_msgs::msg::PoseArray::SharedPtr mock_com_trajectory;
    geometry_msgs::msg::PoseArray::SharedPtr mock_swing_trajectory;

    mock_com_trajectory = std::make_shared<geometry_msgs::msg::PoseArray>();
    mock_swing_trajectory = std::make_shared<geometry_msgs::msg::PoseArray>();

    // before the setters, the check should return false
    ASSERT_EQ(this->ik_buffer->check_if_ready(), false);

    this->ik_buffer->set_com_trajectory(mock_com_trajectory);
    this->ik_buffer->set_swing_trajectory(mock_swing_trajectory);

    ASSERT_EQ(this->ik_buffer->check_if_ready(), true);
    // The check should fail after publishing, so we assert the false statement
    this->ik_buffer->publish_ik_trajectory();
    ASSERT_EQ(this->ik_buffer->check_if_ready(), false);
}

#endif
