////// Copyright 2019 Project March.
#include "march_safety/input_device_safety.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

#include <chrono>

const int DURATION_MS {5000};

using namespace std::chrono_literals;

InputDeviceSafety::InputDeviceSafety(SafetyNode* node, std::shared_ptr<SafetyHandler> safety_handler):
  node_(node),
  safety_handler_(safety_handler),
  connection_timeout_(0)
{
  int milliseconds;
  node->get_parameter("input_device_connection_timeout", milliseconds);
  connection_timeout_ = rclcpp::Duration(std::chrono::milliseconds(milliseconds));
  subscriber_input_device_alive_ = node_->create_subscription<AliveMsg>(
      "/march/input_device/alive", 10, std::bind(&InputDeviceSafety::inputDeviceAliveCallback, this, std::placeholders::_1));
}

void InputDeviceSafety::inputDeviceAliveCallback(const march_shared_msgs::msg::Alive::SharedPtr msg)
{
  this->last_alive_stamps_[msg->id] = msg->stamp;
}

void InputDeviceSafety::update()
{
  auto clock = node_->get_clock();
  auto now = clock->now();
  if (this->last_alive_stamps_.empty())
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock, DURATION_MS, "No input device connected yet");
    return;
  }

  const bool had_connections = !this->connected_devices_.empty();

  for (const auto& last_alive_stamp : this->last_alive_stamps_)
  {
    const std::string& id = last_alive_stamp.first;
    const rclcpp::Time& last_alive = last_alive_stamp.second;
    const bool timed_out = now > (last_alive + this->connection_timeout_);
    const bool is_connected = this->connected_devices_.find(id) != this->connected_devices_.end();

    if (is_connected && timed_out)
    {
      this->connected_devices_.erase(id);
      if (id == "crutch" && !this->connected_devices_.empty())
      {
        this->safety_handler_->publishNonFatal("Crutch input device lost", now);
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Input device `" << id << "` lost");
      }
      else
      {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Input device `" << id << "` lost");
      }
    }
    else if (!is_connected && !timed_out)
    {
      this->connected_devices_.insert(id);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Input device `" << id << "` reconnected. Total connected is "
                                       << this->connected_devices_.size());
    }

    // Check if the alive msg is not timestamped with a future time.
    // This can happen when one node is using sim_time and others aren't.
    // Add small margin to take the stamp offset between board and PC into account
    if (now + rclcpp::Duration(0.5s) < last_alive)
    {
      // Duration is in ms here
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *clock, DURATION_MS, "Input device `" << id << "` alive message is from the future. Current time is "
                                                     << now.seconds() << " and last alive message was "
                                                     << last_alive.seconds());
    }
  }

  const bool has_connections = !this->connected_devices_.empty();
  if (had_connections && !has_connections)
  {
    this->safety_handler_->publishNonFatal("All input devices are lost", now);
  }

  if (!has_connections)
  {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(node_->get_logger(), *clock, DURATION_MS, "No input device connected");
  }
}
