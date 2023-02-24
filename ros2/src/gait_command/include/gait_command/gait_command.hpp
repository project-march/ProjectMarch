//
// Created by march on 23-2-23.
//

#ifndef BUILD_GAIT_COMMAND_HPP
#define BUILD_GAIT_COMMAND_HPP
#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/msg/ipd_input.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include <chrono>
#include <cstdlib>
#include <memory>

enum class InputDevices { Monitor, Eeg, Wireless_ipd };

class GaitCommand {
public:
    GaitCommand();

private:
};
#endif // BUILD_GAIT_COMMAND_HPP
