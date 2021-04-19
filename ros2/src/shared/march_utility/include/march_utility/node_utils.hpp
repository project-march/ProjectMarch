// Copyright 2020 Project March.
#ifndef MARCH_UTILITY_NODE_UTILS_HPP
#define MARCH_UTILITY_NODE_UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>

namespace node_utils {
// Get the Joint names from the robot information node.
std::vector<std::string> get_joint_names(rclcpp::Node& node);
} // namespace node_utils

#endif // MARCH_UTILITY_NODE_UTILS_HPP
