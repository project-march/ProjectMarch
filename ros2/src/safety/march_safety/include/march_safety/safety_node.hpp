// Copyright 2020 Project MARCH
#ifndef MARCH_SAFETY_SAFETY_NODE_H
#define MARCH_SAFETY_SAFETY_NODE_H

#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include <string>
#include <vector>

#include "march_shared_msgs/msg/gait_request.hpp"
#include <march_shared_msgs/msg/error.hpp>
#include <march_shared_msgs/msg/gait_instruction.hpp>

class SafetyNode : public rclcpp::Node {
    // using JointNames = std::vector<std::string>;
    // using ErrorMsg = march_shared_msgs::msg::Error;
    // using ErrorPublisher = rclcpp::Publisher<ErrorMsg>::SharedPtr;
    // using GaitInstruction = march_shared_msgs::msg::GaitInstruction;
    // using GaitInstructionPublisher = rclcpp::Publisher<GaitInstruction>::SharedPtr;

public:
    SafetyNode();

private:
    void startTimer();
    void errorCallback(const march_shared_msgs::msg::Error::SharedPtr msg);
    rclcpp::Subscription<march_shared_msgs::msg::Error>::SharedPtr error_subscriber;  
    rclcpp::TimerBase::SharedPtr timer;
    void update();
};

#endif // MARCH_SAFETY_SAFETY_NODE_H