#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include "rclcpp/rclcpp.hpp"
#include <gtest/gtest.h>

/**
 * The main method which runs all the tests
 */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// NOLINTEND
#endif
