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

#include "march_hardware/motor_controller/odrive/odrive_state.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "march_system_interface/hwi_util.h"
#include "march_system_interface/march_mock_system_interface.hpp"
#include "march_utility/logger_colors.hpp"
#include <csignal>

namespace march_system_interface {

const std::string MarchMockSystemInterface::COMMAND_AND_STATE_TYPE = hardware_interface::HW_IF_POSITION;

// NOLINTNEXTLINE(hicpp-member-init).
MarchMockSystemInterface::MarchMockSystemInterface()
    : logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("MarchMockSystemInterface")))
{
    march_system_interface_util::go_to_stop_state_on_crash(this);
}

/** \brief This should ensure that it goes to the stop state when the instance is being deleted.
 *  \note This doesn't work for thrown exceptions this is why we still call
 *  `march_system_interface_util::go_to_stop_state_on_crash(this);` in the constructor.
 */
MarchMockSystemInterface::~MarchMockSystemInterface()
{
    // NOLINT because this is intended. It needs to calls its own implementation, not that from its child class.
    stop(); // NOLINT(clang-analyzer-optin.cplusplus.VirtualCall)
}

/** Configures the controller.
 * Checkout https://design.ros2.org/articles/node_lifecycle.html, for more information on the execution order.
 */
hardware_interface::return_type MarchMockSystemInterface::configure(const hardware_interface::HardwareInfo& info)
{
    //    info.joints[0].parameters
    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    for (const auto& joint : info.joints) {
        HwStateInfo hw_info;
        hw_info.name = joint.name.c_str();
        hw_info.hw_position = std::numeric_limits<double>::quiet_NaN();
        hw_info.hw_velocity = std::numeric_limits<double>::quiet_NaN();
        hw_info.hw_effort = std::numeric_limits<double>::quiet_NaN();
        hw_state_info_.push_back(hw_info);
    }

    motor_controllers_data_.resize(info_.joints.size(), march::ODriveState());
    RCLCPP_INFO(rclcpp::get_logger("MarchMockSystemInterface"), "%s-----Here!!---", LColor::BLUE);
    if (!march_system_interface_util::joints_have_interface_types(
            info.joints, { COMMAND_AND_STATE_TYPE }, { COMMAND_AND_STATE_TYPE }, (*logger_))) {
        return hardware_interface::return_type::ERROR;
    }
    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

/** Returns a vector of the StateInterfaces.
 *
 * This method is implemented so that the mujoco reader node can publish joint positions.
 * It does this by getting a pointer to the vector containing the positions.
 *
 * In this case this is the same as the vector containing the position_command. Meaning that the broadcaster controller
 * will say that the joints are in the exact positions position controller wants them to be.
 */
std::vector<hardware_interface::StateInterface> MarchMockSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        // Position: Couples the state controller to the value jointInfo.position through a pointer.
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_info_[i].hw_position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_info_[i].hw_velocity));
        // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_info_[i].hw_effort));

        //      [TO DO] For now only a position state interface is created, later when more control types are added,
        //      this should be expanded.

        //        // Velocity: Couples the state controller to the value jointInfo.velocity through a pointer.
        //        state_interfaces.emplace_back(hardware_interface::StateInterface(
        //            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &HwStateInfo.hw_velocity));
        //        // Effort: Couples the state controller to the value jointInfo.velocity through a pointer.
        //        state_interfaces.emplace_back(hardware_interface::StateInterface(
        //            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &HwStateInfo.hw_effort));
    }
    //    for (uint i = 0; i < info_.joints.size(); i++) {
    //        state_interfaces.emplace_back(
    //                hardware_interface::StateInterface(info_.joints[i].name, COMMAND_AND_STATE_TYPE,
    //                &hw_positions_[i]));
    //    }

    return state_interfaces;
}

/** Returns a vector of the CommandInterfaces.
 *
 * This method is implemented so that the position controller can set the calculated positions.
 * It does this by getting a pointer to the vector containing the positions_commands.
 *
 * In this case this is the same as the vector containing the position state. Meaning that the broadcaster controller
 * will say that the joints are in the exact positions position controller wants them to be.
 */
std::vector<hardware_interface::CommandInterface> MarchMockSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(info_.joints[i].name, COMMAND_AND_STATE_TYPE, &hw_positions_[i]);
    }
    return command_interfaces;
}

/// This method is ran when you start the controller, (configure is ran earlier).
hardware_interface::return_type MarchMockSystemInterface::start()
{
    RCLCPP_INFO((*logger_), "Mock System interface Starting ...please wait...");

    // set some default values when starting the first time
    for (uint i = 0; i < hw_positions_.size(); i++) {
        if (std::isnan(hw_positions_[i])) {
            hw_positions_[i] = 0;
        }
    }
    for (auto hw_info : hw_state_info_) {
        if (std::isnan(hw_info.hw_position)) {
            hw_info.hw_position = 0;
        }
        if (std::isnan(hw_info.hw_velocity)) {
            hw_info.hw_velocity = 0;
        }
        if (std::isnan(hw_info.hw_effort)) {
            hw_info.hw_effort = 0;
        }
    }

    comms = std::make_shared<SimCommunication>();
    executor_.add_node(comms);
    std::thread([this]() {
        executor_.spin();
    }).detach();

    status_ = hardware_interface::status::STARTED;

    RCLCPP_INFO((*logger_), "Mock System interface successfully started!, This should go well");

    return hardware_interface::return_type::OK;
}

/// This method is ran when you stop the controller, (start is ran earlier).
hardware_interface::return_type MarchMockSystemInterface::stop()
{
    status_ = hardware_interface::status::STOPPED;
    RCLCPP_INFO((*logger_), "Mock System interface successfully stopped!");
    return hardware_interface::return_type::OK;
}

/** This is the update loop of the state interface.
 *
 *  This method is empty in this case as we directly set the state interface to read from the mujoco reader node.
 *  See: export_state_interfaces and export_command_interfaces().
 */
hardware_interface::return_type MarchMockSystemInterface::read()
{
    auto new_pos = comms->get_pos();
    for (size_t i = 0; i < new_pos.size(); i++) {
        hw_state_info_[i].hw_position = new_pos.at(i);
    }
    auto new_vel = comms->get_pos();
    for (size_t i = 0; i < new_pos.size(); i++) {
        hw_state_info_[i].hw_velocity = new_pos.at(i);
    }
    auto new_eff = comms->get_pos();
    for (size_t i = 0; i < new_pos.size(); i++) {
        hw_state_info_[i].hw_effort = new_eff.at(i);
    }
    return hardware_interface::return_type::OK;
}

/** This is the update loop of the command interface.
 *
 *  This method is empty in this case as we directly set the state interface to read from the mujoco reader node.
 *  See: export_state_interfaces and export_command_interfaces().
 */
hardware_interface::return_type MarchMockSystemInterface::write()
{
    return hardware_interface::return_type::OK;
}

} // namespace march_system_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_system_interface::MarchMockSystemInterface, hardware_interface::SystemInterface)