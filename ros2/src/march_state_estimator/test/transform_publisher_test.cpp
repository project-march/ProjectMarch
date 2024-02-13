//
// Created by AlexanderJamesBecoy on 2023-01-10.
//

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "march_state_estimator/transform_broadcaster.hpp"

class TransformBroadcasterTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        m_node = std::make_shared<TransformBroadcaster>();
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    std::shared_ptr<TransformBroadcaster> m_node;
};

TEST_F(TransformBroadcasterTest, test_should)
{
    // Arrange
    // Act
    // Assert
    ASSERT_TRUE(true);
}

// NOLINTEND
#endif // __clang_analyzer__