// Copyright 2019 Project March.
#ifndef MARCH_SAFETY_TEMPERATURE_SAFETY_H
#define MARCH_SAFETY_TEMPERATURE_SAFETY_H

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "march_shared_msgs/msg/error.hpp"

#include "march_safety/safety_type.hpp"
#include "march_safety/safety_handler.hpp"
#include "march_safety/safety_node.hpp"

enum ThresholdType
{
  FATAL,
  NON_FATAL,
  WARNING
};

class TemperatureSafety : public SafetyType
{
  using TemperatureMsg = sensor_msgs::msg::Temperature;
  using TemperatureSubscription = rclcpp::Subscription<TemperatureMsg>::SharedPtr;
  using ThresholdHoldsMap = std::map<std::string, double>;
public:
  TemperatureSafety(std::shared_ptr<SafetyNode> node, std::shared_ptr<SafetyHandler> safety_handler);

  void update() override { };

private:
  // Callback for when a temperature is published.
  void temperatureCallback(const TemperatureMsg::SharedPtr msg, const std::string& sensor_name);

  // Create a subscriber for each joint on the /march/temperature/<joint> topic
  void createSubscribers();

  // Create an error string for when a temperature is too high.
  std::string getErrorMessage(double temperature, const std::string& sensor_name);

  // Get the threshold of a joint in a thresholdsmap. If there is none, fall back to the default.
  double getThreshold(const std::string& sensor_name, ThresholdHoldsMap temperature_thresholds_map);

  // Set all temperature thresholds.
  void setTemperatureThresholds();

  // Node to use for logging and getting time
  std::shared_ptr<SafetyNode> node_;

  // Safety handlers to use when logging errors
  std::shared_ptr<SafetyHandler> safety_handler_;

  double default_temperature_threshold_;

  rclcpp::Duration send_errors_interval_;
  rclcpp::Time time_last_send_error_;

  // Map of ThresholdHoldsMaps to store threshold values for each joint
  std::map<ThresholdType, ThresholdHoldsMap> thresholds_maps_;

  std::vector<TemperatureSubscription> temperature_subscribers_ = {};
};

#endif  // MARCH_SAFETY_TEMPERATURE_SAFETY_H
