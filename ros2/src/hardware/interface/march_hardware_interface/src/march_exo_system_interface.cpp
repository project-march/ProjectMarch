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

#include "march_hardware_builder/hardware_builder.h"
#include "march_logger_cpp/ros_logger.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "march_hardware_interface/hwi_util.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace march_hardware_interface_util;

namespace march_hardware_interface {

MarchExoSystemInterface::MarchExoSystemInterface() :
        logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("MarchExoSystemInterface"))),
        last_read_time_(std::chrono::steady_clock::now())
{}

/** Configures the controller.
* Checkout https://design.ros2.org/articles/node_lifecycle.html, for more information on the execution order.
 * After On_Configure all state interfaces and "non-movement" command interfaces should be available to controllers
*/
hardware_interface::return_type MarchExoSystemInterface::configure(const hardware_interface::HardwareInfo& info)
{
    RCLCPP_INFO((*logger_), "Configuring Hardware Interface...");

    // Default Check needs to be done for every hardware interface.
    if (configure_default(info) != hardware_interface::return_type::OK) {
        RCLCPP_FATAL((*logger_), "Configure default of Hardware Interface failed.");
        return hardware_interface::return_type::ERROR;
    }

    // Checks if the joints have the correct command and state interfaces (if not check you controller.yaml).
    if (!joints_have_interface_types(
            /*joints=*/info.joints,
            /*required_command_interfaces=*/{ hardware_interface::HW_IF_EFFORT },
            /*required_state_interfaces=*/{hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY},
            /*logger=*/(*logger_))) {
        return hardware_interface::return_type::ERROR;
    }

    // Makes the virtual hardware objects to handle the ethercat communication with the hardware.
    try {
        march_robot_ = load_march_hardware(info);
        RCLCPP_INFO((*logger_), "Finished creating march hardware. With %i joints. ", march_robot_->size());
    } catch (const std::exception& e) {
        RCLCPP_FATAL((*logger_), "Something went wrong in the making of march hardware see: \n\t%s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    joints_info_.reserve(info_.joints.size());
    for (const auto& joint : info.joints) {

        JointInfo jointInfo {
            /*name=*/joint.name.c_str(),
            /*joint=*/march_robot_->getJoint(joint.name.c_str()),
            /*position=*/std::numeric_limits<double>::quiet_NaN(),
            /*velocity=*/std::numeric_limits<double>::quiet_NaN(),
            /*effort_command=*/std::numeric_limits<double>::quiet_NaN(),
            /*max_effort=*/stod(get_parameter(joint, "max_effort", "30")),
            /*max_velocity=*/stod(get_parameter(joint, "max_velocity", "3.5"))};
        if(!has_correct_actuation_mode(jointInfo.joint)){
            return hardware_interface::return_type::ERROR;
        }
        joints_info_.push_back(jointInfo);
        RCLCPP_INFO((*logger_), "Joint: %s, has '%g' max_effort and a max_velocity of '%g'.",
                    joint.name.c_str(), jointInfo.max_effort, jointInfo.max_velocity);
    }

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

/** Returns a vector of the StateInterfaces.
*
* This method is implemented so that the joint_state_broadcaster controller can publish joint positions.
* It does this by getting a pointer to the variable that stores the positions in this class.
*
* In this case this is the same as the vector containing the position_command. Meaning that the broadcaster controller
* will say that the joints are in the exact positions position controller wants them to be.
*/
std::vector<hardware_interface::StateInterface> MarchExoSystemInterface::export_state_interfaces()
{
    RCLCPP_INFO((*logger_), "Creating export state interface.");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (JointInfo jointInfo : joints_info_) {
        // Position: Couples the state controller to the value jointInfo.position through a pointer.
        state_interfaces.emplace_back(
                hardware_interface::StateInterface(jointInfo.name, hardware_interface::HW_IF_POSITION,
                                                   &jointInfo.position));
        // Velocity: Couples the state controller to the value jointInfo.velocity through a pointer.
        state_interfaces.emplace_back(
                hardware_interface::StateInterface(jointInfo.name, hardware_interface::HW_IF_VELOCITY,
                                                   &jointInfo.velocity));
    }
    return state_interfaces;
}

/** Returns a vector of the CommandInterfaces.
*
* This method is implemented so that the position controller can set the calculated positions.
* It does this by getting a pointer to the variable that stores the effort_commands in this class.
*
* In this case this is the same as the vector containing the position state. Meaning that the broadcaster controller
* will say that the joints are in the exact positions position controller wants them to be.
*/
std::vector<hardware_interface::CommandInterface> MarchExoSystemInterface::export_command_interfaces()
{
    RCLCPP_INFO((*logger_), "Creating export command interface.");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (JointInfo jointInfo : joints_info_) {
        // Effort: Couples the command controller to the value jointInfo.effort through a pointer.
        command_interfaces.emplace_back(
                hardware_interface::CommandInterface(jointInfo.name, hardware_interface::HW_IF_EFFORT,
                                                   &jointInfo.effort_command));
    }

    return command_interfaces;
}

/// This method is ran when you start the controller (especially command controller), (configure is ran earlier).
hardware_interface::return_type MarchExoSystemInterface::start()
{
    // Start ethercat cycle in the hardware
    RCLCPP_INFO((*logger_), "Starting EthercatCycle...");
    march_robot_->startEtherCAT(/*reset_motor_controllers=*/false);
    auto jointPtrs = march_robot_->getJoints();
    RCLCPP_INFO((*logger_), "Waiting for Joints to sent ethercat data...");
    repeat_function_on_joints_until_timeout(
            /*function_goal=*/"Waiting on ethercat data.",
            /*function=*/[](march::Joint& joint) {return joint.getMotorController()->getState()->dataIsValid();},
            /*logger=*/(*logger_), /*joints=*/jointPtrs,
            /*function_when_timeout=*/[this](march::Joint& joint) {
                RCLCPP_ERROR((*logger_), "Joints %s is not receiving data",
                             joint.getName().c_str());
            });
    RCLCPP_INFO((*logger_), "All slaves are sending EtherCAT data.");

    // Read the first encoder values for each joint
    for (JointInfo jointInfo : joints_info_) {
        jointInfo.joint.readFirstEncoderValues(/*operational_check/=*/false);

        // Set the first target as the current position
        jointInfo.position = jointInfo.joint.getPosition();
        jointInfo.velocity = 0;
        jointInfo.effort_command = 0;
    }

    RCLCPP_INFO((*logger_), "All joints are ready for reading!");
    status_ = hardware_interface::status::STARTED;
    return hardware_interface::return_type::OK;
}

/** \brief This method is used to switch command modes, but we use it to activate the command options on the hardware.
*
* By handling making the hardware actuation ready in this step we can make sure to,
* read values of the joints (while they are not even actuation ready).
*
* @param start_interfaces The new starting interfaces.
* @param stop_interfaces  The interfaces that are stopping.
* @return hardware_interface::return_type::OK if everything is correct,
*          hardware_interface::return_type::ERROR otherwise.
*/
hardware_interface::return_type
MarchExoSystemInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                     const std::vector<std::string> &stop_interfaces) {
    for (const auto& start: start_interfaces) {
        RCLCPP_INFO((*logger_), "Starting interfaces: %s", start.c_str());
    }
    for (const auto& stop: stop_interfaces) {
        RCLCPP_INFO((*logger_), "Stopping interfaces: %s", stop.c_str());
    }

    make_joints_operational(march_robot_->getNotOperationalJoints());

    return hardware_interface::return_type::OK;
}

void MarchExoSystemInterface::make_joints_operational(std::vector<march::Joint*> joints) const {
    if (joints.empty()) { return; }

    // Tell every MotorController to clear its errors.
    call_function_and_wait_on_joints(
            /*function_goal=*/"Clearing errors of joints.",
            /*function=*/[](march::Joint& joint) {return joint.getMotorController()->reset();},
            /*logger=*/(*logger_), /*joints=*/joints);

    // Prepare every joint for actuation.
    call_function_and_wait_on_joints(
            /*function_goal=*/"Preparing joints for actuation.",
            /*function=*/[](march::Joint& joint) {return joint.prepareActuation();},
            /*logger=*/(*logger_), /*joints=*/joints);

    // Tell every joint to enable actuation.
    // TODO: Check if this needs be done for every joint or the non operational once.
    RCLCPP_INFO((*logger_), "Enabling every joint for actuation");
    for (auto joint : joints) {
        joint->enableActuation();
    }

    repeat_function_on_joints_until_timeout(
            /*function_goal=*/"Check if joints are  in operational state.",
            /*function=*/[](march::Joint& joint) {return joint.getMotorController()->getState()->isOperational();},
            /*logger=*/(*logger_), /*joints=*/joints,
            /*function_when_timeout=*/[this](march::Joint& joint) {
                RCLCPP_ERROR((*logger_), "Joint %s is not an operational state.",
                             joint.getName().c_str());
            });
}

/// This method is ran when you stop the controller, (start is ran earlier).
hardware_interface::return_type MarchExoSystemInterface::stop()
{
    // Stopping ethercat cycle in the hardware
    RCLCPP_INFO((*logger_), "Stopping EthercatCycle...");
    march_robot_->stopEtherCAT();
    RCLCPP_INFO((*logger_), "EthercatCycle successfully stopped.");
    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
}

/** This is the update loop of the state interface.
*
*  This method is empty in this case as we directly set the state interface to read from the command controller.
*  See: export_state_interfaces and export_command_interfaces().
*/
hardware_interface::return_type MarchExoSystemInterface::read()
{
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_between_last_update = current_time - last_read_time_;
    last_read_time_ = std::chrono::steady_clock::now();

    for(JointInfo jointInfo : joints_info_) {
        jointInfo.joint.readEncoders(time_between_last_update);
        jointInfo.position = jointInfo.joint.getPosition();
        jointInfo.velocity = jointInfo.joint.getVelocity();
    }
    return hardware_interface::return_type::OK;
}

/** This is the update loop of the command interface.
*
*  This method is empty in this case as we directly set the state interface to read from the command controller.
*  See: export_state_interfaces and export_command_interfaces().
*/
hardware_interface::return_type MarchExoSystemInterface::write()
{
    return hardware_interface::return_type::OK;
}

std::unique_ptr<march::MarchRobot> MarchExoSystemInterface::load_march_hardware(const hardware_interface::HardwareInfo &info) const {
    auto pos_iterator = info.hardware_parameters.find("robot");
    if (pos_iterator == info.hardware_parameters.end()) {
        throw std::invalid_argument("Hardware Parameter 'robot' not specified. Check under your "
                                    "'<hardware>' tag in the 'control/xacro/ros2_control.xacro' file.");
    }
    string robot_config_file_path = ament_index_cpp::get_package_share_directory("march_hardware_builder") +
            '/' + "robots" + '/' + pos_iterator->second + ".yaml";

    RCLCPP_INFO((*logger_), "Robot config file path: %s", robot_config_file_path.c_str());


    HardwareBuilder hw_builder {
        /*yaml_path=*/robot_config_file_path,
        /*logger=*/shared_ptr<march_logger::RosLogger>(new march_logger::RosLogger(logger_))};

    std::vector<std::string> joint_names;
    joint_names.reserve(info.joints.size());
    for (const auto& joint : info.joints) {
        joint_names.emplace_back(joint.name.c_str());
    }

    return hw_builder.createMarchRobot(/*active_joint_names=*/joint_names);
}

bool MarchExoSystemInterface::has_correct_actuation_mode(march::Joint& joint) const {
    const auto& actuation_mode = joint.getMotorController()->getActuationMode();
    if (actuation_mode != march::ActuationMode::torque) {
        RCLCPP_FATAL((*logger_), "Actuation mode for joint %s is not torque, but is %s.\n "
                                 "Check your `march_hardware_builder/robots/... .yaml`.",
                     joint.getName().c_str(), actuation_mode.toString().c_str());
        return false;
    }
    return true;
}


} // namespace march_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_hardware_interface::MarchExoSystemInterface, hardware_interface::SystemInterface)
