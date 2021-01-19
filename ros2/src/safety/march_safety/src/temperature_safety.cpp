// Copyright 2019 Project March.

#include "march_safety/temperature_safety.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"

#include <chrono>
#include <map>
#include <string>
#include <vector>

const double DEFAULT_TEMPERATURE_THRESHOLD {40.0};
const int DEFAULT_SEND_ERRORS_INTERVAL {1000};

// TODO(@Tim) Throw an exception when no temperatures are published.
/**
 * @brief Construct the TemperatureSafety class
 * @param node Node to use for logging
 * @param safety_handler Safety handler to use for publishing errors
 * @param joint_names List of joint names
 * @details Get the required parameters.
 *          Set all temperature thresholds.
 *          Create temperature subscribers.
 */
TemperatureSafety::TemperatureSafety(SafetyNode* node,
                                     std::shared_ptr<SafetyHandler> safety_handler,
                                     std::vector<std::string> joint_names)
  : node_(node),
  safety_handler_(safety_handler),
  send_errors_interval_(0),
  joint_names_(std::move(joint_names))
{
  node_->get_parameter_or("default_temperature_threshold", default_temperature_threshold_, DEFAULT_TEMPERATURE_THRESHOLD);
  setAllTemperatureThresholds();

  int send_errors_interval_ms;
  node_->get_parameter_or("send_errors_interval", send_errors_interval_ms, DEFAULT_SEND_ERRORS_INTERVAL);
  send_errors_interval_ = rclcpp::Duration(std::chrono::milliseconds(send_errors_interval_ms));

  time_last_send_error_ = node_->get_clock()->now() - send_errors_interval_;

  createSubscribers();
}

/**
 * @brief Set all temperature thresholds.
 */
void TemperatureSafety::setAllTemperatureThresholds()
{
  for (auto type : THRESHOLD_TYPES)
  {
    setTemperatureThresholds(type);
  }
}

/**
 * @brief Set all temperature thresholds for a specific type.
 * @param type Type to set thresholds for.
 */
void TemperatureSafety::setTemperatureThresholds(std::string& type)
{
  for (std::string joint : joint_names_)
  {
    double threshold_value;
    std::string parameter_name = "temperature_thresholds_" + type + "." + joint;

    bool parameter_is_set = node_->get_parameter_or(parameter_name, threshold_value, default_temperature_threshold_);
    setThreshold(type, joint, threshold_value);

    if (!parameter_is_set) {
        node_->declare_parameter(parameter_name, default_temperature_threshold_);
    }
  }
}

/**
 * @brief Set a temperature threshold for a specific type and joint.
 * @param type Type to set thresholds for.
 * @param joint Joint to set thresholds for.
 * @param threshold_value Threshold value to set.
 */
void TemperatureSafety::setThreshold(std::string& type, std::string joint, double threshold_value)
{
  // Is there a better way to do this ?
    if (type == "fatal")
    {
      fatal_temperature_thresholds_map_.insert({ joint, threshold_value });
    }
    else if (type == "non_fatal")
    {
      non_fatal_temperature_thresholds_map_.insert({ joint, threshold_value });
    }
    else if (type == "warning")
    {
       warning_temperature_thresholds_map_.insert({ joint, threshold_value });
    }
    else
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown temperature threshold type: " << type);
    }
}

/**
 * @brief Callback for when a temperature is published.
 * @param msg Temperature that is published.
 * @param sensor_name Joint to which the temperature belongs.
 * @details This callback checks if the temperature values do not exceed the defined threshold.
 */
void TemperatureSafety::temperatureCallback(const TemperatureMsg::SharedPtr msg, const std::string& sensor_name)
{
  // Send at most an error every 'send_errors_interval_'
  if (node_->get_clock()->now() <= (time_last_send_error_ + send_errors_interval_))
  {
    return;
  }

  double temperature = msg->temperature;
  if (temperature <= getThreshold(sensor_name, warning_temperature_thresholds_map_))
  {
    return;
  }

  std::string error_message = getErrorMessage(temperature, sensor_name);

  // Sometimes the temperature can have large outliers, which we will ignore.
  // TODO(Olav) this is a temporary fix, this should be fixed locally on the slaves ask Electro.
  if (temperature > 200)
  {
    RCLCPP_WARN(node_->get_logger(), "%s", error_message.c_str());
    return;
  }

  // If the threshold is exceeded raise an error
  if (temperature > getThreshold(sensor_name, fatal_temperature_thresholds_map_))
  {
    safety_handler_->publishFatal(error_message);
  }
  else if (temperature > getThreshold(sensor_name, non_fatal_temperature_thresholds_map_))
  {
    safety_handler_->publishNonFatal(error_message);
  }
  else if (temperature > getThreshold(sensor_name, warning_temperature_thresholds_map_))
  {
    RCLCPP_WARN(node_->get_logger(), "%s", error_message.c_str());
  }
}

/**
 * @brief Create an error string for when a temperature is too high.
 * @param temperature Temperature that is published.
 * @param sensor_name Joint to which the temperature belongs.
 */
std::string TemperatureSafety::getErrorMessage(double temperature, const std::string& sensor_name)
{
  std::ostringstream message_stream;
  message_stream << sensor_name << " temperature too high: " << temperature;
  std::string error_message = message_stream.str();
  return error_message;
}

/**
 * @brief Get the threshold of a joint in a thresholdsmap. If there is none, fall back to the default.
 * @param sensor_name Joint to get temperature threshold of.
 * @param temperature_thresholds_map thresholdsmap to look for joint name.
 */
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

/**
 * @brief Create a subscriber for each joint on the /march/temperature/<joint> topic
 */
void TemperatureSafety::createSubscribers()
{
  for (const std::string& joint_name : joint_names_)
  {
    // Construct callback instead of using std::bind
    // https://github.com/ros2/rclcpp/issues/583#issuecomment-442146657
    auto callback = [this, joint_name](const TemperatureMsg::SharedPtr msg) {
      temperatureCallback(msg, joint_name);
    };
    auto subscriber_temperature = node_->create_subscription<TemperatureMsg>(
        "/march/temperature/" + joint_name, 1000, callback);

    temperature_subscribers_.push_back(subscriber_temperature);
  }
}
