// Copyright 2019 Project March.

#include "march_safety/temperature_safety.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"

#include <chrono>
#include <map>
#include <string>
#include <vector>

const double DEFAULT_TEMPERATURE_THRESHOLD {40.0};

// TODO(@Tim) Throw an exception when no temperatures are published.
TemperatureSafety::TemperatureSafety(SafetyNode* node,
                                     std::shared_ptr<SafetyHandler> safety_handler,
                                     std::vector<std::string> joint_names)
  : node_(node),
  safety_handler_(safety_handler),
  joint_names_(std::move(joint_names))
{
  node_->get_parameter_or("default_temperature_threshold", default_temperature_threshold_, DEFAULT_TEMPERATURE_THRESHOLD);
  setAllTemperatureThresholds();

  node_->get_parameter("send_errors_interval", send_errors_interval_);
  this->time_last_send_error_ = rclcpp::Time(0);

  this->createSubscribers();
}

void TemperatureSafety::setAllTemperatureThresholds()
{
  for (auto type : THRESHOLD_TYPES)
  {
    setTemperatureThresholds(type);
  }
}

void TemperatureSafety::setTemperatureThresholds(std::string& type)
{
  auto temperature_thresholds_map = getThresholdsMap(type);

  for (std::string joint : joint_names_)
  {
    double threshold_value;
    std::string parameter_name = "temperature_thresholds_" + type + "." + joint;

    bool parameter_is_set = node_->get_parameter_or(parameter_name, threshold_value, default_temperature_threshold_);
    temperature_thresholds_map.insert({joint, threshold_value});

    RCLCPP_INFO_STREAM(node_->get_logger(), "Name: " << parameter_name << ", value: " << threshold_value << ", Set:" << parameter_is_set);

    if (!parameter_is_set) {
        node_->declare_parameter(parameter_name, default_temperature_threshold_);
    }
  }
}

TemperatureSafety::ThresholdHoldsMap TemperatureSafety::getThresholdsMap(std::string& type)
{
  // Is there a better way to do this ?
    if (type.compare("fatal") == 0)
        return fatal_temperature_thresholds_map_;
    else if (type.compare("non_fatal") == 0)
        return non_fatal_temperature_thresholds_map_;
    else
        return warning_temperature_thresholds_map_;
}

void TemperatureSafety::temperatureCallback(const TemperatureMsg::SharedPtr msg, const std::string& sensor_name)
{
  // send at most an error every second
  if (node_->get_clock()->now() <= (time_last_send_error_ + rclcpp::Duration(std::chrono::milliseconds(send_errors_interval_))))
  {
    return;
  }

  double temperature = msg->temperature;
  if (temperature <= getThreshold(sensor_name, warning_temperature_thresholds_map_))
  {
    return;
  }

  std::string error_message = getErrorMessage(temperature, sensor_name);

  // TODO(Olav) this is a temporary fix, this should be fixed locally on the slaves ask Electro.
  if (temperature > 200)
  {
    RCLCPP_WARN(node_->get_logger(), "%s", error_message.c_str());
    return;
  }

  auto now = node_->get_clock()->now();
  // If the threshold is exceeded raise an error
  if (temperature > getThreshold(sensor_name, fatal_temperature_thresholds_map_))
  {
    safety_handler_->publishFatal(error_message, now);
  }
  else if (temperature > getThreshold(sensor_name, non_fatal_temperature_thresholds_map_))
  {
    safety_handler_->publishNonFatal(error_message, now);
  }
  else if (temperature > getThreshold(sensor_name, warning_temperature_thresholds_map_))
  {
    RCLCPP_WARN(node_->get_logger(), "%s", error_message.c_str());
  }
}

std::string TemperatureSafety::getErrorMessage(double temperature, const std::string& sensor_name)
{
  std::ostringstream message_stream;
  message_stream << sensor_name << " temperature too high: " << temperature;
  std::string error_message = message_stream.str();
  return error_message;
}

double TemperatureSafety::getThreshold(const std::string& sensor_name, ThresholdHoldsMap temperature_thresholds_map)
{
  if (temperature_thresholds_map.find(sensor_name) != temperature_thresholds_map.end())
  {
    // Return specific defined threshold for this sensor
    return temperature_thresholds_map[sensor_name];
  }
  else
  {
    // Fall back to default if there is no defined threshold
    RCLCPP_WARN_ONCE(node_->get_logger(), "There is a specific temperature threshold missing for %s sensor", sensor_name.c_str());
    return default_temperature_threshold_;
  }
}

void TemperatureSafety::createSubscribers()
{
  for (const std::string& joint_name : joint_names_)
  {
    // Construct callback instead of using std::bind, see https://github.com/ros2/rclcpp/issues/583#issuecomment-442146657
    auto callback = [this, joint_name](const TemperatureMsg::SharedPtr msg) {
      this->temperatureCallback(msg, joint_name);
    };
    auto subscriber_temperature = node_->create_subscription<TemperatureMsg>(
        "/march/temperature/" + joint_name, 1000, callback);

    temperature_subscribers_.push_back(subscriber_temperature);
  }
}
