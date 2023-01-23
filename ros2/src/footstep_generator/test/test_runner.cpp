#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2018 Project March.

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

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
