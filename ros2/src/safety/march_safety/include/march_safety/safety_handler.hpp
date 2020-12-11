// Copyright 2019 Project March.
#ifndef MARCH_SAFETY_SAFETY_HANDLER_H
#define MARCH_SAFETY_SAFETY_HANDLER_H

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "march_safety/safety_node.hpp"

#include <march_shared_msgs/msg/error.hpp>
#include <march_shared_msgs/msg/gait_instruction.hpp>

class SafetyHandler
{
  using ErrorMsg = march_shared_msgs::msg::Error;
  using GaitInstruction = march_shared_msgs::msg::GaitInstruction;
  using ErrorPublisher = rclcpp::Publisher<ErrorMsg>::SharedPtr;
  using GaitInstructionPublisher = rclcpp::Publisher<GaitInstruction>::SharedPtr;
public:
  SafetyHandler(SafetyNode* node,
                ErrorPublisher error_publisher,
                GaitInstructionPublisher gait_instruction_publisher);

  void publishFatal(const std::string& message, const rclcpp::Time& now);

  void publishNonFatal(const std::string& message, const rclcpp::Time& now);

  void publishErrorMessage(const std::string& message, const rclcpp::Time& now, int8_t error_type) const;

  void publishStopMessage(const rclcpp::Time& now) const;

private:
  SafetyNode* node_;
  ErrorPublisher error_publisher_;
  GaitInstructionPublisher gait_instruction_publisher_;
};

#endif  // MARCH_SAFETY_SAFETY_HANDLER_H
