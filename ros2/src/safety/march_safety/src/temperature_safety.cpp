// Copyright 2019 Project March.

#include "march_safety/temperature_safety.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

const double DEFAULT_TEMPERATURE_THRESHOLD { 40.0 };
const int DEFAULT_SEND_ERRORS_INTERVAL { 1000 };

const std::map<ThresholdType, std::string> THRESHOLD_NAMES
    = { { FATAL, /*__y=*/"fatal" }, { NON_FATAL, /*__y=*/"non_fatal" },
          { WARNING, /*__y=*/"warning" } };

// TODO(@Tim) Throw an exception when no temperatures are published.
/**
 * @brief Construct the TemperatureSafety class
 * @param node Node to use for logging
 * @details Get the required parameters.
 *          Set all temperature thresholds.
 *          Create temperature subscribers.
 */
TemperatureSafety::TemperatureSafety(std::shared_ptr<SafetyNode> node,
    std::shared_ptr<SafetyHandler> safety_handler)
    : node_(std::move(node))
    , safety_handler_(std::move(safety_handler))
    , send_errors_interval_(/*nanoseconds=*/0)
    , default_temperature_threshold_(DEFAULT_TEMPERATURE_THRESHOLD)
{
    node_->get_parameter_or("default_temperature_threshold",
        default_temperature_threshold_, DEFAULT_TEMPERATURE_THRESHOLD);

    int send_errors_interval_ms;
    node_->get_parameter_or("send_errors_interval", send_errors_interval_ms,
        DEFAULT_SEND_ERRORS_INTERVAL);
    send_errors_interval_
        = rclcpp::Duration(std::chrono::milliseconds(send_errors_interval_ms));

    time_last_send_error_ = node_->get_clock()->now() - send_errors_interval_;

    setTemperatureThresholds();
    createSubscribers();
}

/**
 * @brief Set all temperature thresholds.
 */
void TemperatureSafety::setTemperatureThresholds()
{
    for (const auto& type : THRESHOLD_NAMES) {
        ThresholdHoldsMap thresholds_map;
        for (const std::string& joint : node_->joint_names) {
            double threshold_value;
            std::string parameter_name
                = "temperature_thresholds_" + type.second + "." + joint;

            bool parameter_is_set = node_->get_parameter_or(parameter_name,
                threshold_value, default_temperature_threshold_);
            thresholds_map.insert({ joint, threshold_value });

            if (!parameter_is_set) {
                node_->declare_parameter(
                    parameter_name, default_temperature_threshold_);
            }
        }
        thresholds_maps_.insert({ type.first, thresholds_map });
    }
}

/**
 * @brief Callback for when a temperature is published.
 * @param msg Temperature that is published.
 * @param sensor_name Joint to which the temperature belongs.
 * @details This callback checks if the temperature values do not exceed the
 * defined threshold.
 */
void TemperatureSafety::temperatureCallback(
    const TemperatureMsg::SharedPtr& msg, const std::string& sensor_name)
{
    // Send at most an error every 'send_errors_interval_'
    if (node_->get_clock()->now()
        <= (time_last_send_error_ + send_errors_interval_)) {
        return;
    }

    double temperature = msg->temperature;
    if (temperature
        <= getThreshold(sensor_name, thresholds_maps_.at(WARNING))) {
        return;
    }

    std::string error_message = getErrorMessage(temperature, sensor_name);

    // Sometimes the temperature can have large outliers, which we will ignore.
    // TODO(Olav) this is a temporary fix, this should be fixed locally on the
    // slaves ask Electro.
    if (temperature > 200) {
        RCLCPP_WARN(node_->get_logger(), "%s", error_message.c_str());
        return;
    }

    // If the threshold is exceeded raise an error
    if (temperature > getThreshold(sensor_name, thresholds_maps_.at(FATAL))) {
        safety_handler_->publishFatal(error_message);
    } else if (temperature
        > getThreshold(sensor_name, thresholds_maps_.at(NON_FATAL))) {
        safety_handler_->publishNonFatal(error_message);
    } else if (temperature
        > getThreshold(sensor_name, thresholds_maps_.at(WARNING))) {
        RCLCPP_WARN(node_->get_logger(), "%s", error_message.c_str());
    }
}

/**
 * @brief Create an error string for when a temperature is too high.
 * @param temperature Temperature that is published.
 * @param sensor_name Joint to which the temperature belongs.
 */
std::string TemperatureSafety::getErrorMessage(
    double temperature, const std::string& sensor_name)
{
    std::ostringstream message_stream;
    message_stream << sensor_name << " temperature too high: " << temperature;
    std::string error_message = message_stream.str();
    return error_message;
}

/**
 * @brief Get the threshold of a joint in a thresholdsmap. If there is none,
 * fall back to the default.
 * @param sensor_name Joint to get temperature threshold of.
 * @param temperature_thresholds_map thresholdsmap to look for joint name.
 */
double TemperatureSafety::getThreshold(const std::string& sensor_name,
    ThresholdHoldsMap temperature_thresholds_map)
{
    if (temperature_thresholds_map.find(sensor_name)
        != temperature_thresholds_map.end()) {
        // Return specific defined threshold for this sensor
        return temperature_thresholds_map[sensor_name];
    } else {
        // Fall back to default if there is no defined threshold
        RCLCPP_WARN_ONCE(node_->get_logger(),
            "There is a specific temperature threshold missing for %s sensor",
            sensor_name.c_str());
        return default_temperature_threshold_;
    }
}

/**
 * @brief Create a subscriber for each joint on the /march/temperature/<joint>
 * topic
 */
void TemperatureSafety::createSubscribers()
{
    for (const std::string& joint_name : node_->joint_names) {
        // Construct callback instead of using std::bind
        // https://github.com/ros2/rclcpp/issues/583#issuecomment-442146657
        auto callback
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            = [this, joint_name](const TemperatureMsg::SharedPtr msg) {
                  temperatureCallback(msg, joint_name);
              };
        auto subscriber_temperature
            = node_->create_subscription<TemperatureMsg>(
                "/march/temperature/" + joint_name, 1000, callback);

        temperature_subscribers_.push_back(subscriber_temperature);
    }
}
