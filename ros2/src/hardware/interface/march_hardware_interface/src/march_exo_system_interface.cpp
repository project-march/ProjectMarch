// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "march_hardware_interface/march_exo_system_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace march_hardware_interface
{
hardware_interface::return_type MarchExoSystemInterface::configure(
        const hardware_interface::HardwareInfo & info)
{
    if (configure_default(info) != hardware_interface::return_type::OK)
    {
        return hardware_interface::return_type::ERROR;
    }

    hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            for (const auto& command_interface : joint.command_interfaces) {
                if (command_interface.name != hardware_interface::HW_IF_EFFORT) {
                    RCLCPP_ERROR(rclcpp::get_logger("MarchExoSystemInterface"),
                                 "Joint '%s' has command interfaces '%s', expected: '%s'.",
                                 joint.name.c_str(), command_interface.name.c_str(), hardware_interface::HW_IF_EFFORT);
                }
            }

            RCLCPP_FATAL(
                    rclcpp::get_logger("MarchExoSystemInterface"),
                    "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("MarchExoSystemInterface"),
                    "Joint '%s' has '%s' command interfaces, expected '%s'.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("MarchExoSystemInterface"),
                    "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("MarchExoSystemInterface"),
                    "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("MarchExoSystemInterface"),
                    "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::return_type::ERROR;
        }
    }

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
MarchExoSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MarchExoSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_position_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type MarchExoSystemInterface::start()
{
    RCLCPP_INFO(rclcpp::get_logger("MarchExoSystemInterface"), "Starting ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
                rclcpp::get_logger("MarchExoSystemInterface"), "%.1f seconds left...",
                hw_start_sec_ - i);
    }

    // set some default values when starting the first time
    for (uint i = 0; i < hw_positions_.size(); i++)
    {
        if (std::isnan(hw_positions_[i]))
        {
            hw_positions_[i] = 0;
            hw_velocities_[i] = 0;
            hw_position_commands_[i] = 0;
        }
    }

    status_ = hardware_interface::status::STARTED;

    RCLCPP_INFO(
            rclcpp::get_logger("MarchExoSystemInterface"), "System Successfully started!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarchExoSystemInterface::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("MarchExoSystemInterface"), "Stopping ...please wait...");

    for (int i = 0; i < hw_stop_sec_; i++)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(
                rclcpp::get_logger("MarchExoSystemInterface"), "%.1f seconds left...",
                hw_stop_sec_ - i);
    }

    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(
            rclcpp::get_logger("MarchExoSystemInterface"), "System successfully stopped!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarchExoSystemInterface::read()
{
    RCLCPP_INFO(rclcpp::get_logger("MarchExoSystemInterface"), "Reading...");

    for (uint i = 0; i < hw_positions_.size(); i++)
    {
        if (hw_positions_[i] <= 0.8) {
            hw_positions_[i] += 0.01;
        } else{
            hw_positions_[i] = 0.2;
        }
        // Simulate RRBot's movement
//      hw_positions_[i] = hw_positions_[i] + (hw_position_commands_[i] - hw_positions_[i]) / hw_slowdown_;
        RCLCPP_INFO(rclcpp::get_logger("MarchExoSystemInterface"), "Got state %.5f for joint %d!",
                    hw_positions_[i], i);
    }
    RCLCPP_INFO(rclcpp::get_logger("MarchExoSystemInterface"), "Joints successfully read!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarchExoSystemInterface::write()
{
    RCLCPP_INFO(rclcpp::get_logger("MarchExoSystemInterface"), "Writing...");

    for (uint i = 0; i < hw_position_commands_.size(); i++)
    {
//        hw_position_commands_[i] = hw_positions_[i];
        // Simulate sending commands to the hardware
        RCLCPP_INFO(
                rclcpp::get_logger("MarchExoSystemInterface"), "Got command %.5f for joint %d!",
                hw_position_commands_[i], i);
    }
    RCLCPP_INFO(
            rclcpp::get_logger("MarchExoSystemInterface"), "Joints successfully written!");

    return hardware_interface::return_type::OK;
}

}  // namespace march_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    march_hardware_interface::MarchExoSystemInterface, hardware_interface::SystemInterface)