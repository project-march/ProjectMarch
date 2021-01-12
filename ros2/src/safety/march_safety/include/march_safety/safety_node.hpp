// Copyright 2020 Project MARCH
#ifndef MARCH_SAFETY_SAFETY_NODE_H
#define MARCH_SAFETY_SAFETY_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/timer.hpp"

#include "march_safety/safety_type.hpp"

#include <vector>
#include <string>


class SafetyNode : public rclcpp::Node {
  using JointNames = std::vector<std::string>;
  public:
    SafetyNode(
        const std::string& node_name,
        const std::string& node_namespace
    );

    void start();
    void update();

  private:
    std::vector<std::unique_ptr<SafetyType>> safety_list;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif  // MARCH_SAFETY_SAFETY_NODE_H