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

#include "march_hardware_interface/march_rviz_system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "march_hardware_interface/hwi_util.h"
#include "march_utility/logger_colors.hpp"

namespace march_hardware_interface {

const std::string MarchRvizSystemInterface::COMMAND_AND_STATE_TYPE = hardware_interface::HW_IF_POSITION;

void MarchRvizSystemInterface::initSim(std::vector<MjcStateInfo> mjc_state_info){
    RCLCPP_INFO((*logger_), "Hardware interface starts communication with mujoco!", LColor::GREEN);
    std::vector<MjcStateInfo> vec = mjc_state_info;
    for (int i = 0; i < static_cast<int>(vec.size()); i++ ){
        hw_state_info_[i].mjc_state_info = vec[i];
    }
    updateHwState();
}

void MarchRvizSystemInterface::updateHwState(){
    for (int i; i < static_cast<int>(hw_state_info_.size()); i++){
        auto mjc_info = hw_state_info_[i].mjc_state_info;
        hw_state_info_[i].name = mjc_info.name;
        hw_state_info_[i].hw_position = mjc_info.mjc_position;
        hw_state_info_[i].hw_velocity = mjc_info.mjc_velocity;
        hw_state_info_[i].hw_effort = mjc_info.mjc_effort;
    }
}

// NOLINTNEXTLINE(hicpp-member-init) The pdb_data_ should be initialized at the configure step.
MarchRvizSystemInterface::MarchRvizSystemInterface()
    : logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("MarchRvizSystemInterface")))
{
    march_hardware_interface_util::go_to_stop_state_on_crash(this);
}

/** \brief This should ensure that it goes to the stop state when the instance is being deleted.
 *  \note This doesn't work for thrown exceptions this is why we still call
 *  `march_hardware_interface_util::go_to_stop_state_on_crash(this);` in the constructor.
 */
MarchRvizSystemInterface::~MarchRvizSystemInterface()
{
    // NOLINT because this is intended. It needs to calls its own implementation, not that from its child class.
    stop(); // NOLINT(clang-analyzer-optin.cplusplus.VirtualCall)
}

/** Configures the controller.
 * Checkout https://design.ros2.org/articles/node_lifecycle.html, for more information on the execution order.
 */
hardware_interface::return_type MarchRvizSystemInterface::configure(const hardware_interface::HardwareInfo& info)
{
    //    info.joints[0].parameters
    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    motor_controllers_data_.resize(info_.joints.size(), march::ODriveState());
    RCLCPP_INFO(rclcpp::get_logger("MarchRvizSystemInterface"), "%s-----Here!!---", LColor::BLUE);
    if (!march_hardware_interface_util::joints_have_interface_types(
            info.joints, { COMMAND_AND_STATE_TYPE }, { COMMAND_AND_STATE_TYPE }, (*logger_))) {
        return hardware_interface::return_type::ERROR;
    }
    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

/** Returns a vector of the StateInterfaces.
 *
 * This method is implemented so that the joint_state_broadcaster controller can publish joint positions.
 * It does this by getting a pointer to the vector containing the positions.
 *
 * In this case this is the same as the vector containing the position_command. Meaning that the broadcaster controller
 * will say that the joints are in the exact positions position controller wants them to be.
 */
std::vector<hardware_interface::StateInterface> MarchRvizSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        // Position: Couples the state controller to the value jointInfo.position through a pointer.
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_info_[i].hw_position));
//        // Velocity: Couples the state controller to the value jointInfo.velocity through a pointer.
//        state_interfaces.emplace_back(hardware_interface::StateInterface(
//            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &HwStateInfo.hw_velocity));
//        // Effort: Couples the state controller to the value jointInfo.velocity through a pointer.
//        state_interfaces.emplace_back(hardware_interface::StateInterface(
//            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &HwStateInfo.hw_effort));

//        state_interfaces.emplace_back(
//            hardware_interface::StateInterface(info_.joints[i].name, COMMAND_AND_STATE_TYPE, &hw_positions_[i]));

        for (std::pair<std::string, double*>& motor_controller_pointer : motor_controllers_data_[i].get_pointers()) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, motor_controller_pointer.first, motor_controller_pointer.second));
        }
    }
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
std::vector<hardware_interface::CommandInterface> MarchRvizSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(info_.joints[i].name, COMMAND_AND_STATE_TYPE, &hw_positions_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type MarchRvizSystemInterface::perform_command_mode_switch(
    const vector<std::string>& start_interfaces, const vector<std::string>& stop_interfaces)
{
    for (const auto& start : start_interfaces) {
        RCLCPP_INFO((*logger_), "Start interfaces: %s", start.c_str());
    }
    for (const auto& stop : stop_interfaces) {
        RCLCPP_INFO((*logger_), "Stop interfaces: %s", stop.c_str());
    }
    return hardware_interface::return_type::OK;
}

/// This method is ran when you start the controller, (configure is ran earlier).
hardware_interface::return_type MarchRvizSystemInterface::start()
{
    RCLCPP_INFO((*logger_), "HW Rviz System interface Starting ...please wait...");

    // set some default values Wwhen starting the first time
    for (uint i = 0; i < hw_positions_.size(); i++) {
        if (std::isnan(hw_positions_[i])) {
            hw_positions_[i] = 0;
        }
    }
    status_ = hardware_interface::status::STARTED;
    RCLCPP_INFO((*logger_), "HW Rviz System interface successfully started!, This should go wel");

    return hardware_interface::return_type::OK;
}

/// This method is ran when you stop the controller, (start is ran earlier).
hardware_interface::return_type MarchRvizSystemInterface::stop()
{
    status_ = hardware_interface::status::STOPPED;
    RCLCPP_INFO((*logger_), "HW Rviz System interface successfully stopped!");
    return hardware_interface::return_type::OK;
}

/** This is the update loop of the state interface.
 *
 *  This method is empty in this case as we directly set the state interface to read from the command controller.
 *  See: export_state_interfaces and export_command_interfaces().
 */
hardware_interface::return_type MarchRvizSystemInterface::read()
{
//    // Here the hw_positions should be updated
//    for (uint i = 0; i < hw_positions_.size(); i++) {
//        hw_positions_[i] = 1;
//    }
    return hardware_interface::return_type::OK;
}

/** This is the update loop of the command interface.
 *
 *  This method is empty in this case as we directly set the state interface to read from the command controller.
 *  See: export_state_interfaces and export_command_interfaces().
 */
hardware_interface::return_type MarchRvizSystemInterface::write()
{
    updateHwState();
    return hardware_interface::return_type::OK;
}

} // namespace march_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_hardware_interface::MarchRvizSystemInterface, hardware_interface::SystemInterface)