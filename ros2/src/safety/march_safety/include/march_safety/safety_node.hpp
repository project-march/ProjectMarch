// Copyright 2020 Project MARCH
#ifndef MARCH_SAFETY_SAFETY_NODE_H
#define MARCH_SAFETY_SAFETY_NODE_H

#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "march_safety/safety_type.hpp"

#include <string>
#include <vector>

#include <march_shared_msgs/msg/error.hpp>
#include <march_shared_msgs/msg/gait_instruction.hpp>

class SafetyNode : public rclcpp::Node {
    using JointNames = std::vector<std::string>;
    using ErrorMsg = march_shared_msgs::msg::Error;
    using ErrorPublisher = rclcpp::Publisher<ErrorMsg>::SharedPtr;
    using GaitInstruction = march_shared_msgs::msg::GaitInstruction;
    using GaitInstructionPublisher
        = rclcpp::Publisher<GaitInstruction>::SharedPtr;

public:
    SafetyNode();

    // Start the safety node
    void start();

    // Update the safety listeners
    void update();

    // Public attributes
    JointNames joint_names;
    std::vector<std::unique_ptr<SafetyType>> safety_list;

    ErrorPublisher error_publisher;
    GaitInstructionPublisher gait_instruction_publisher;

private:
    rclcpp::TimerBase::SharedPtr timer;
};

#endif // MARCH_SAFETY_SAFETY_NODE_H