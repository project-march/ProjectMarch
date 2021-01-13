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

const std::string THRESHOLD_TYPES[] = {"fatal", "non_fatal", "warning"};

class TemperatureSafety : public SafetyType
{
  using TemperatureMsg = sensor_msgs::msg::Temperature;
  using TemperatureSubscription = rclcpp::Subscription<TemperatureMsg>::SharedPtr;
  using ThresholdHoldsMap = std::map<std::string, double>;
public:
  TemperatureSafety(SafetyNode* node, std::shared_ptr<SafetyHandler> safety_handler, std::vector<std::string> joint_names);

  void update() override { };

private:
  /**
   * This callback checks if the temperature values do not exceed the defined threshold
   * @param msg the temperature message
   * @param sensor_name the name of the sender
   */
  void temperatureCallback(const TemperatureMsg::SharedPtr msg, const std::string& sensor_name);

  /**
   * Create a subscriber for every sensor that publishes temperatures
   */
  void createSubscribers();

  std::string getErrorMessage(double temperature, const std::string& sensor_name);

  /**
   * Find the specific defined threshold for this sensor
   * If there is none, fall back to the default
   * @param sensor_name
   * @return the threshold
   */
  double getThreshold(const std::string& sensor_name, ThresholdHoldsMap temperature_thresholds_map);

  void setAllTemperatureThresholds();
  void setTemperatureThresholds(std::string& type);
  void setThreshold(std::string& type, std::string joint, double threshold_value);

  SafetyNode* node_;
  std::shared_ptr<SafetyHandler> safety_handler_;
  double default_temperature_threshold_;
  rclcpp::Duration send_errors_interval_;
  rclcpp::Time time_last_send_error_;

  ThresholdHoldsMap fatal_temperature_thresholds_map_;
  ThresholdHoldsMap non_fatal_temperature_thresholds_map_;
  ThresholdHoldsMap warning_temperature_thresholds_map_;

  std::vector<TemperatureSubscription> temperature_subscribers_ = {};
  std::vector<std::string> joint_names_;
};

#endif  // MARCH_SAFETY_TEMPERATURE_SAFETY_H
