#include "rclcpp/rclcpp.hpp"
#include <gtest/gtest.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
