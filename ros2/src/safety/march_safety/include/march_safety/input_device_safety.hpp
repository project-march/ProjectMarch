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
  InputDeviceSafety(std::shared_ptr<SafetyNode> node, std::shared_ptr<SafetyHandler> safety_handler);

  // Update the input device safety checker.
  void update() override;

private:
  // Callback for when the input device publishes on the /march/input_device/alive topic
  void inputDeviceAliveCallback(const AliveMsg::SharedPtr msg);

  // Check the last alive stamp of an input device.
  void check_last_alive_stamp(const std::string& id, const rclcpp::Time& last_alive);

  // Node to use for logging and getting time
  std::shared_ptr<SafetyNode> node_;

  // Safety handlers to use when logging errors
  std::shared_ptr<SafetyHandler> safety_handler_;

  // Subscription for the /march/input_device/alive topic
  AliveSubscription subscriber_input_device_alive_;

  // Time to wait before a connection is considered timed out
  rclcpp::Duration connection_timeout_;

  // Map containing for each input device the last time they published to the /march/input_device/alive topic
  std::unordered_map<std::string, rclcpp::Time> last_alive_stamps_;

  // Set of connected devices
  std::unordered_set<std::string> connected_devices_;
};

#endif  // MARCH_SAFETY_INPUT_DEVICE_SAFETY_H
