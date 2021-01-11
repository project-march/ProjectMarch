// Copyright 2020 Project MARCH
#ifndef MARCH_SAFETY_SAFETY_NODE_H
#define MARCH_SAFETY_SAFETY_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"

#include "march_safety/safety_type.hpp"

#include <vector>
#include <string>


class SafetyNode : public rclcpp::Node {
  using JointNames = std::vector<std::string>;
  public:
    SafetyNode(
        const std::string& node_name,
        const rclcpp::NodeOptions& options
    );

    void start(const double update_rate);

  private:
    std::vector<std::unique_ptr<SafetyType>> safety_list;
};

#endif  // MARCH_SAFETY_SAFETY_NODE_H