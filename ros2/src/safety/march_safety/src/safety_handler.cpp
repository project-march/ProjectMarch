//// Copyright 2019 Project March.
#include "rclcpp/rclcpp.hpp"

#include "march_safety/safety_handler.hpp"
#include "march_safety/safety_node.hpp"
#include <march_shared_msgs/msg/error.hpp>
#include <march_shared_msgs/msg/gait_instruction.hpp>

#include <string>


SafetyHandler::SafetyHandler(SafetyNode* node,
                             ErrorPublisher error_publisher,
                             GaitInstructionPublisher gait_instruction_publisher)
  : node_(node)
  , error_publisher_(error_publisher)
  , gait_instruction_publisher_(gait_instruction_publisher)
{
}

void SafetyHandler::publishErrorMessage(const std::string& message, const rclcpp::Time& now, int8_t error_type) const
{
  ErrorMsg error_msg;
  std::ostringstream message_stream;
  error_msg.header.stamp = now;
  error_msg.error_message = message;
  error_msg.type = error_type;
  this->error_publisher_->publish(error_msg);
}

void SafetyHandler::publishStopMessage(const rclcpp::Time& now) const
{
  GaitInstruction gait_instruction_msg;
  gait_instruction_msg.header.stamp = now;
  gait_instruction_msg.type = GaitInstruction::STOP;
  this->gait_instruction_publisher_->publish(gait_instruction_msg);
}

void SafetyHandler::publishFatal(const std::string& message, const rclcpp::Time& now)
{
  RCLCPP_ERROR(this->node_->get_logger(),"%s", message.c_str());

  this->publishErrorMessage(message, now, ErrorMsg::FATAL);
}

void SafetyHandler::publishNonFatal(const std::string& message, const rclcpp::Time& now)
{
  RCLCPP_ERROR(this->node_->get_logger(),"%s", message.c_str());

  this->publishStopMessage(now);
  this->publishErrorMessage(message, now, ErrorMsg::NON_FATAL);
}
