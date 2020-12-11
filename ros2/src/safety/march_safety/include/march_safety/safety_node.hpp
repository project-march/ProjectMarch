// Copyright 2020 Project MARCH
#ifndef MARCH_SAFETY_SAFETY_NODE_H
#define MARCH_SAFETY_SAFETY_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"

#include <vector>
#include <string>


class SafetyNode final : public rclcpp::Node {
  using JointNames = std::vector<std::string>;
  public:
    SafetyNode(
        const std::string& node_name,
        const rclcpp::NodeOptions& options
    );

  private:
    // Retrieve the joint names from the published robot description.
    JointNames get_joint_names();
};

#endif  // MARCH_SAFETY_SAFETY_NODE_H