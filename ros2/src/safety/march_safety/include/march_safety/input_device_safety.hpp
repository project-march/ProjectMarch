//// Copyright 2019 Project March
#ifndef MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
#define MARCH_SAFETY_INPUT_DEVICE_SAFETY_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

#include "march_safety/safety_type.hpp"
#include "march_safety/safety_handler.hpp"
#include "march_safety/safety_node.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <march_shared_msgs/msg/alive.hpp>
#include <march_shared_msgs/msg/error.hpp>

class InputDeviceSafety : public SafetyType
{
  using AliveMsg = march_shared_msgs::msg::Alive;
  using AliveSubscription = rclcpp::Subscription<AliveMsg>::SharedPtr;
public:
  InputDeviceSafety(SafetyNode* node, std::shared_ptr<SafetyHandler> safety_handler);

  void update() override;

private:
  void inputDeviceAliveCallback(const AliveMsg::SharedPtr msg);

  SafetyNode* node_;

  std::shared_ptr<SafetyHandler> safety_handler_;

  AliveSubscription subscriber_input_device_alive_;

  rclcpp::Duration connection_timeout_;

  std::unordered_map<std::string, rclcpp::Time> last_alive_stamps_;
  std::unordered_set<std::string> connected_devices_;
};

#endif  // MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
