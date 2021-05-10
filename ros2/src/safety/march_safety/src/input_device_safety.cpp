////// Copyright 2019 Project March.
#include "march_safety/input_device_safety.hpp"

#include <string>

#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <utility>

using namespace std::chrono_literals;

const float THROTTLE_DURATION_MS { 5000.0 };

/**
 * @file input_device_safety.cpp
 * @brief Responsible for publishing error message regarding the input device
 */

/**
 * @param node Reference to the safety node
 */
InputDeviceSafety::InputDeviceSafety(std::shared_ptr<SafetyNode> node,
    std::shared_ptr<SafetyHandler> safety_handler)
    : node_(std::move(node))
    , safety_handler_(std::move(safety_handler))
    , connection_timeout_(/*nanoseconds=*/0)
{
    int milliseconds;
    node_->get_parameter("input_device_connection_timeout", milliseconds);

    connection_timeout_
        = rclcpp::Duration(std::chrono::milliseconds(milliseconds));

    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    auto callback = [this](const AliveMsg::SharedPtr msg) {
        inputDeviceAliveCallback(msg);
    };
    subscriber_input_device_alive_ = node_->create_subscription<AliveMsg>(
        "/march/input_device/alive", 10, callback);
}

/**
 * @brief Callback for when the input device publishes on the
 * /march/input_device/alive topic
 * @param msg Alive msg
 */
void InputDeviceSafety::inputDeviceAliveCallback(const AliveMsg::SharedPtr& msg)
{
    last_alive_stamps_[msg->id] = msg->stamp;
}

/**
 * @brief Update the input device safety checker.
 * @details This function looks at the last_alive_stamps_ map.
 *          If there are no stamps yet, an information message is printed to the
 * screen and the function is exited. Then for every input device connected,
 * their last_alive_stamp is checked. If there were connections before checking
 * and are no connections after checking, then all input devices are lost and a
 * non fatal error is published.
 */
void InputDeviceSafety::update()
{
    if (last_alive_stamps_.empty()) {
        // Log the info message only once every 'THROTTLE_DURATION_MS'
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(),
            THROTTLE_DURATION_MS, "No input device connected yet");
        return;
    }

    const bool had_connections = !connected_devices_.empty();

    for (const auto& last_alive_stamp : last_alive_stamps_) {
        check_last_alive_stamp(last_alive_stamp.first, last_alive_stamp.second);
    }

    const bool has_connections = !connected_devices_.empty();
    if (had_connections && !has_connections) {
        safety_handler_->publishNonFatal("All input devices are lost");
    }

    if (!has_connections) {
        RCLCPP_INFO_SKIPFIRST_THROTTLE(node_->get_logger(), *node_->get_clock(),
            THROTTLE_DURATION_MS, "No input device connected");
    }
}

/**
 * @brief Check the last alive stamp of an input device.
 * @param id Id of the input device
 * @param last_alive Time to check
 * @details If the input device is connected but timed_out, then the input
 * device is actually no longer connected. The input device is then removed from
 * the connected_devices.
 *
 *          If the input device is not connected ye and also not timed out, then
 * a new input device is connected and the input device is added to the set of
 * connected_devices
 */
void InputDeviceSafety::check_last_alive_stamp(
    const std::string& id, const rclcpp::Time& last_alive)
{
    auto now = node_->get_clock()->now();

    const bool timed_out = now > (last_alive + connection_timeout_);
    const bool is_connected
        = connected_devices_.find(id) != connected_devices_.end();

    if (is_connected && timed_out) {
        connected_devices_.erase(id);
        if (id == "crutch" && !connected_devices_.empty()) {
            safety_handler_->publishNonFatal("Crutch input device lost");
            RCLCPP_ERROR_STREAM(
                node_->get_logger(), "Input device `" << id << "` lost");
        } else {
            RCLCPP_WARN_STREAM(
                node_->get_logger(), "Input device `" << id << "` lost");
        }
    } else if (!is_connected && !timed_out) {
        connected_devices_.insert(id);
        RCLCPP_INFO_STREAM(node_->get_logger(),
            "Input device `" << id << "` reconnected. Total connected is "
                             << connected_devices_.size());
    }

    // Check if the alive msg is not timestamped with a future time.
    // This can happen when one node is using sim_time and others aren't.
    // Add small margin to take the stamp offset between board and PC into
    // account
    // NOLINTNEXTLINE(bugprone-argument-comment)
    if (now + rclcpp::Duration(/*duration=*/0.5s) < last_alive) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(),
            THROTTLE_DURATION_MS,
            "Input device `"
                << id << "` alive message is from the future. Current time is "
                << now.seconds() << " and last alive message was "
                << last_alive.seconds());
    }
}
