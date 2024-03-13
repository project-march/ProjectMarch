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
//#include "march_hardware_interface/weight_node.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <unistd.h>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_interface/hwi_util.h"
#include "march_logger_cpp/ros_logger.hpp"
#include "march_utility/logger_colors.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>

#define DEBUG
#define TORQUEDEBUG

using namespace march_hardware_interface_util;

namespace march_hardware_interface {

// NOLINTNEXTLINE(hicpp-member-init) The pdb_data_ should be initialized at the configure step.
MarchExoSystemInterface::MarchExoSystemInterface()
    : logger_(std::make_shared<rclcpp::Logger>(rclcpp::get_logger("HardwareInterface")))
    , clock_(rclcpp::Clock())
//    , m_weight_node_()
{
    RCLCPP_INFO((*logger_), "creating Hardware Interface...");
    //    m_weight_node_->m_hardware_interface = this;
    //    RCLCPP_INFO((*logger_), "should've assigned the hwi to the weightnode now...");
    go_to_stop_state_on_crash(this); // Note this doesn't work if the ethercat connection is lost.
}

/** \brief This should ensure that it goes to the stop state when the instance is being deleted.
 *  \note This doesn't work for thrown exceptions this is why we still call
 *  `march_hardware_interface_util::go_to_stop_state_on_crash(this);` in the constructor.
 */
MarchExoSystemInterface::~MarchExoSystemInterface()
{
    // NOLINT because this is intended. It needs to calls its own implementation, not that from its child class.
    stop(); // NOLINT(clang-analyzer-optin.cplusplus.VirtualCall)
}

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
            /*required_command_interfaces=*/ { hardware_interface::HW_IF_POSITION },
            /*required_state_interfaces=*/
            { hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY },
            /*logger=*/(*logger_))) {
        RCLCPP_FATAL((*logger_), "Joints do not have the right interface types");
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
    pdb_data_ = {};

    for (const auto& joint : info.joints) {
        JointInfo jointInfo = build_joint_info(joint);
        if (!has_correct_actuation_mode(jointInfo.joint)) {
            return hardware_interface::return_type::ERROR;
        }
        joints_info_.push_back(jointInfo);
        RCLCPP_INFO((*logger_), "Joint: %s, has '%g' max effort difference.", joint.name.c_str(),
            jointInfo.limit.max_effort_differance);
    }

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

/** \brief Builds a JointInfo object uses parameters for some value.
 * \note Most of the defaults param values are set in the ros2_control xacro in package `march_control`.
 */
JointInfo MarchExoSystemInterface::build_joint_info(const hardware_interface::ComponentInfo& joint)
{

    string stop_hardl_param = get_parameter(joint, "stop_when_outside_hard_limits", "true");
    bool stop_when_outside_hard_limits
        = !(stop_hardl_param == "false" or stop_hardl_param == "0" or stop_hardl_param == "False");
    if (stop_when_outside_hard_limits
        and !(stop_hardl_param == "true" or stop_hardl_param == "1" or stop_hardl_param == "True")) {
        RCLCPP_WARN((*logger_),
            "Joint: %s, got param '%s' for `stop_when_outside_hard_limits` but expected "
            "['true', 'false', '0', '1', 'True', 'False']. Defaulting to True.",
            joint.name.c_str(), stop_hardl_param.c_str());
    }
    return { /*name=*/joint.name.c_str(),
        /*joint=*/march_robot_->getJoint(joint.name.c_str()),
        /*motor_controller_data=*/march::ODriveState(),
        /*position=*/std::numeric_limits<double>::quiet_NaN(),
        /*target_position=*/std::numeric_limits<double>::quiet_NaN(),
        /*velocity=*/std::numeric_limits<double>::quiet_NaN(),
        /*torque=*/std::numeric_limits<double>::quiet_NaN(),
        /*target_torque=*/std::numeric_limits<double>::quiet_NaN(),
        /*effort_actual=*/std::numeric_limits<double>::quiet_NaN(),
        /*effort_command=*/std::numeric_limits<double>::quiet_NaN(),
        /*effort_command_converted=*/std::numeric_limits<double>::quiet_NaN(),
        /*position_weight=*/std::numeric_limits<double>::quiet_NaN(),
        /*torque_weight=*/std::numeric_limits<double>::quiet_NaN(),
        /*limit=*/
        JointLimit {
            /*soft_limit_warning_throttle_msec=*/stoi(get_parameter(joint, "soft_limit_warning_throttle_msec", "1500")),
            /*last_time_not_in_soft_error_limit=*/std::chrono::steady_clock::now(),
            /*msec_until_error_when_in_error_soft_limits=*/
            std::chrono::milliseconds {
                stoi(get_parameter(joint, "msec_until_error_when_in_error_soft_limits", "1000")) },
            /*soft_error_limit_warning_throttle_msec=*/
            stoi(get_parameter(joint, "soft_error_limit_warning_throttle_msec", "300")),
            /*max_effort_differance=*/stod(get_parameter(joint, "max_effort_differance", "10")),
            /*stop_when_outside_hard_limits=*/stop_when_outside_hard_limits } };
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
    for (JointInfo& jointInfo : joints_info_) {
        // Position: Couples the state controller to the value jointInfo.position through a pointer.
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            jointInfo.name, hardware_interface::HW_IF_POSITION, &jointInfo.position));
        // Velocity: Couples the state controller to the value jointInfo.velocity through a pointer.
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            jointInfo.name, hardware_interface::HW_IF_VELOCITY, &jointInfo.velocity));
        // Effort: Couples the state controller to the value jointInfo.velocity through a pointer.
        // state_interfaces.emplace_back(
        //     hardware_interface::StateInterface(jointInfo.name, hardware_interface::HW_IF_EFFORT, &jointInfo.torque));
        // For motor controller state broadcasting.
        for (std::pair<std::string, double*>& motor_controller_pointer :
            jointInfo.motor_controller_data.get_pointers()) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                jointInfo.name, motor_controller_pointer.first, motor_controller_pointer.second));
        }
    }

    // // For the PDB broadcaster.
    for (std::pair<std::string, double*>& pdb_pointer : pdb_data_.get_pointers()) {
        state_interfaces.emplace_back(hardware_interface::StateInterface("PDB", pdb_pointer.first, pdb_pointer.second));
    }

    RCLCPP_INFO((*logger_), "Creating export state interface finished.");
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
    for (JointInfo& jointInfo : joints_info_) {
        RCLCPP_INFO((*logger_), "Creating command interface for joint %s", jointInfo.name.c_str());
        // Effort: Couples the command controller to the value jointInfo.target_torque through a pointer.
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     jointInfo.name, hardware_interface::HW_IF_EFFORT, &jointInfo.target_torque));
        // Position: Couples the command controller to the value jointInfo.target_position through a pointer.
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            jointInfo.name, hardware_interface::HW_IF_POSITION, &jointInfo.target_position));
    }

    return command_interfaces;
}

/// This method is ran when you start the controller (especially command controller), (configure is ran earlier).
hardware_interface::return_type MarchExoSystemInterface::start()
{
    try {
        // Start ethercat cycle in the hardware
        // RCLCPP_INFO((*logger_), "Starting EthercatCycle...");
        march_robot_->startEtherCAT(/*reset_motor_controllers=*/false);
        auto jointPtrs = march_robot_->getJoints();
        // RCLCPP_INFO((*logger_), "Waiting for Joints to sent ethercat data...");
        repeat_function_on_joints_until_timeout(
            /*function_goal=*/"Waiting on ethercat data.",
            /*function=*/
            [](march::Joint& joint) {
                return joint.getMotorController()->getState()->dataIsValid();
            },
            /*logger=*/(*logger_), /*joints=*/jointPtrs,
            /*function_when_timeout=*/
            [this](march::Joint& joint) {
                RCLCPP_ERROR((*logger_), "Joints %s is not receiving data", joint.getName().c_str());
            });
        RCLCPP_INFO((*logger_), "All slaves are sending EtherCAT data.");

        // Read the first encoder values for each joint
        for (JointInfo& jointInfo : joints_info_) {
            // Send PID values to the joints to initialize them
            jointInfo.joint.sendPID();
            // RCLCPP_INFO((*logger_), "Set PID's for joint %s", jointInfo.name.c_str());

            // TODO: examine why this check is set to false.
            jointInfo.joint.readFirstEncoderValues(/*operational_check/=*/true);
            jointInfo.target_position = (float)jointInfo.joint.getPosition();
            jointInfo.target_torque = jointInfo.joint.getTorque();

            RCLCPP_WARN((*logger_), "The first read pos value is %f", jointInfo.target_position);
            RCLCPP_WARN((*logger_), "The first set torque value is %f", jointInfo.target_torque);

            // if no weight has been assigned, we start in position control
            if (!jointInfo.torque_weight || isnan(jointInfo.torque_weight) || !jointInfo.position_weight
                || isnan(jointInfo.position_weight)) {
                jointInfo.torque_weight = 0.0f;
                jointInfo.position_weight = 1.0f;
            }

            RCLCPP_WARN((*logger_),
                "The fuzzy target values for the %s are as follows: \n target position: %f \n measured position: %f \n position "
                "weight: %f \n target torque: %f \n measured torque: %f \n torque weight: %f",jointInfo.joint.getName().c_str(),
                jointInfo.target_position, jointInfo.position, jointInfo.position_weight, jointInfo.target_torque,
                jointInfo.torque, jointInfo.torque_weight);

            // jointInfo.joint.actuate(
            //     jointInfo.target_position, jointInfo.target_torque, jointInfo.position_weight, jointInfo.torque_weight);

            // Set the first target as the current position
            jointInfo.position = jointInfo.joint.getPosition();
            jointInfo.velocity = 0;
            jointInfo.torque = jointInfo.joint.getTorque();
            jointInfo.effort_actual = 0;
            jointInfo.effort_command = 0;

        }
        weight_node = std::make_shared<WeightNode>();
        weight_node->joints_info_ = getJointsInfo();
        executor_.add_node(weight_node);
        // gains_node = std::make_shared<GainsNode>();
        // gains_node->joints_info_ = getJointsInfo();
        // executor_.add_node(gains_node);
        std::thread([this]() {
            executor_.spin();
        }).detach();

    } catch (const std::exception& e) {
        RCLCPP_FATAL((*logger_), e.what());
        throw;
    }

    RCLCPP_INFO((*logger_), "%sAll joints are ready for reading!", LColor::BLUE);
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
hardware_interface::return_type MarchExoSystemInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
    RCLCPP_INFO((*logger_), "%sStart writing on!", LColor::BLUE);
    for (const auto& start : start_interfaces) {
        RCLCPP_INFO((*logger_), "Starting interfaces: %s", start.c_str());
    }
    for (const auto& stop : stop_interfaces) {
        RCLCPP_INFO((*logger_), "Stopping interfaces: %s", stop.c_str());
    }
    try {
        if (!start_interfaces.empty()) {
            make_joints_operational(march_robot_->getNotOperationalJoints());
            joints_ready_for_actuation_ = true;
            RCLCPP_INFO((*logger_), "%sAll joints ready for writing.", LColor::GREEN);
            for (JointInfo& jointInfo : joints_info_) {
                jointInfo.limit.last_time_not_in_soft_error_limit = std::chrono::steady_clock::now();
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL((*logger_), e.what());
        stop();
        throw;
    }

    return hardware_interface::return_type::OK;
}

void MarchExoSystemInterface::make_joints_operational(std::vector<march::Joint*> joints)
{
    if (joints.empty()) {
        return;
    }

    // Tell every MotorController to clear its errors.
    call_function_and_wait_on_joints(
        /*function_goal=*/"Clearing errors of joints.",
        /*function=*/
        [](march::Joint& joint) {
            return joint.getMotorController()->reset();
        },
        /*logger=*/(*logger_), /*joints=*/joints);

    // Prepare every joint for actuation.
    call_function_and_wait_on_joints(
        /*function_goal=*/"Preparing joints for actuation.",
        /*function=*/
        [](march::Joint& joint) {
            return joint.prepareActuation();
        },
        /*logger=*/(*logger_), /*joints=*/joints);

    // Tell every joint to enable actuation.
    // TODO: Check if this needs be done for every joint or the non operational once.
    RCLCPP_INFO((*logger_), "Enabling every joint for actuation");
    for (auto joint : joints) {
        joint->enableActuation();
    }

    repeat_function_on_joints_until_timeout(
        /*function_goal=*/"Check if joints are in operational state.",
        /*function=*/
        [](march::Joint& joint) {
            return joint.getMotorController()->getState()->isOperational();
        },
        /*logger=*/(*logger_), /*joints=*/joints,
        /*function_when_timeout=*/
        [this](march::Joint& joint) {
            RCLCPP_ERROR((*logger_),
                "Joint %s is not an operational state."
                "\n\t\tExpected state '%s' but got state '%s'",
                joint.getName().c_str(),
                march::ODriveAxisState::toString(march::ODriveAxisState::CLOSED_LOOP_CONTROL).c_str(),
                joint.getMotorController()->getState()->getOperationalState().c_str());
        });
}

/// This method is ran when you stop the controller, (start is ran earlier).
hardware_interface::return_type MarchExoSystemInterface::stop()
{
    // Stopping ethercat cycle in the hardware
    RCLCPP_INFO_ONCE((*logger_), "Stopping EthercatCycle...");
    for (JointInfo& jointInfo : joints_info_) {
        // control on zero output torque when the exo shuts down.
        // RCLCPP_INFO(rclcpp::get_logger(jointInfo.joint.getName().c_str()), "Position is: %f", jointInfo.position);
        jointInfo.joint.actuate((float)jointInfo.position, /*torque=*/0.0f, 1.0f, 0.0f);
    }
    joints_ready_for_actuation_ = false;
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
    if (!is_ethercat_alive(this->march_robot_->getLastEthercatException(), (*logger_))) {
        // This is necessary as in ros foxy return::type error does not yet bring it to a stop (which it should).
        throw runtime_error("Ethercat is not alive!");
        return hardware_interface::return_type::ERROR;
    }
    // Wait for the ethercat train to be back.
    this->march_robot_->waitForPdo();

    for (JointInfo& jointInfo : joints_info_) {
        jointInfo.joint.readEncoders();
        jointInfo.position = jointInfo.joint.getPosition();
        jointInfo.velocity = jointInfo.joint.getVelocity();
        jointInfo.torque = jointInfo.joint.getTorque();
        jointInfo.effort_actual = jointInfo.joint.getMotorController()->getActualEffort();
        jointInfo.motor_controller_data.update_values(jointInfo.joint.getMotorController()->getState().get());

        // RCLCPP_INFO(rclcpp::get_logger(jointInfo.joint.getName().c_str()), "Position is: %f", jointInfo.position);
    }
    return hardware_interface::return_type::OK;
}

/**
 * @brief Reads the pdb data from the hardware and updates it so that the broadcaster can publish it.
 * Raises warnings if the voltage goes below a certain value.
 * The data is published on `/march/pdb_data`.
 */
void MarchExoSystemInterface::pdb_read()
{
    march_robot_->getPowerDistributionBoard().read(pdb_data_);
    if (pdb_data_.battery_voltage != 0) {
        if (pdb_data_.battery_voltage < 40) {
            RCLCPP_ERROR_THROTTLE(
                (*logger_), clock_, 500, "Battery voltage is less then 40V, it is: %gV.", pdb_data_.battery_voltage);
        } else if (pdb_data_.battery_voltage < 45) {
            RCLCPP_WARN_THROTTLE(
                (*logger_), clock_, 1000, "Battery voltage is less then 45V, it is: %gV.", pdb_data_.battery_voltage);
        }
    }
}

/** This is the update loop of the command interface.
 *
 *  This method is empty in this case as we directly set the state interface to read from the command controller.
 *  See: export_state_interfaces and export_command_interfaces().
 */
hardware_interface::return_type MarchExoSystemInterface::write()
{
    // When the joints are not yet ready don't write anything to them.
    if (!joints_ready_for_actuation_) {
        return hardware_interface::return_type::OK;
    }
    
    // publish the measured torque each iteration
    weight_node->publish_measured_torque();

    for (JointInfo& jointInfo : joints_info_) {
        if (!is_joint_in_valid_state(jointInfo)) {
            // This is necessary as in ros foxy return::type error does not yet bring it to a stop (which it should).
            throw runtime_error("Joint not in valid state!");
        }

// either this or setting the target torque to the real-time measured torque in the weight node
// jointInfo.target_torque = jointInfo.joint.getMotorController()->getTorqueSensor()->getAverageTorque();

// TORQUEDEBUG LINE

        // ACTUAL TORQUE LINE
    // jointInfo.joint.actuate((float)jointInfo.target_position, (float)jointInfo.target_torque,
    //     (float)jointInfo.position_weight, (float)jointInfo.torque_weight);
    // }

    RCLCPP_WARN((*logger_),
        "Actuation target values for the %s are as follows: \n target position: %f \n measured position: %f \n position "
        "weight: %f \n target torque: %f \n measured torque: %f \n torque weight: %f",jointInfo.joint.getName().c_str(),
        jointInfo.target_position, jointInfo.position, jointInfo.position_weight, jointInfo.target_torque,
        jointInfo.torque, jointInfo.torque_weight);
    }

    RCLCPP_INFO_ONCE((*logger_), "%sActuation has started!",LColor::BLUE);
    return hardware_interface::return_type::OK;
}

// Checks whether the joint and its motor controller are in a valid state.
bool MarchExoSystemInterface::is_joint_in_valid_state(JointInfo& jointInfo)
{
    if (jointInfo.position == 0) {
        RCLCPP_WARN((*logger_), "The joint %s has position 0, the absolute encoder probably isn't working correctly.",
            jointInfo.name.c_str());
    }
    return is_motor_controller_in_a_valid_state(jointInfo.joint, (*logger_)) && !is_joint_in_limit(jointInfo);
}

bool MarchExoSystemInterface::is_joint_in_limit(JointInfo& jointInfo)
{
    // SOFT Limit check.
    if (jointInfo.joint.isWithinSoftLimits()) {
        jointInfo.limit.last_time_not_in_soft_error_limit = std::chrono::steady_clock::now();
        return false;
    }
    const auto& abs_encoder = jointInfo.joint.getMotorController()->getAbsoluteEncoder();
    const double joint_pos_rad = jointInfo.joint.getPosition();
    const int joint_pos_iu = abs_encoder->positionRadiansToIU(joint_pos_rad);

    RCLCPP_WARN_THROTTLE((*logger_), clock_, jointInfo.limit.soft_limit_warning_throttle_msec,
        "Joint %s outside its soft limits [%i, %i] at (%g rad; %i IU)", jointInfo.name.c_str(),
        abs_encoder->getLowerSoftLimitIU(), abs_encoder->getUpperSoftLimitIU(), joint_pos_rad, joint_pos_iu);

    // ERROR Soft Limit Check
    if (jointInfo.joint.isWithinSoftErrorLimits()) {
        jointInfo.limit.last_time_not_in_soft_error_limit = std::chrono::steady_clock::now();
        return false;
    }
    auto time_in_error_limits = (std::chrono::steady_clock::now() - jointInfo.limit.last_time_not_in_soft_error_limit);
    if (jointInfo.limit.msec_until_error_when_in_error_soft_limits > 0s
        && time_in_error_limits > jointInfo.limit.msec_until_error_when_in_error_soft_limits) {
        RCLCPP_FATAL((*logger_),
            "Joint %s (%g rad; %i IU) is outside its soft ERROR limits [%i, %i], for %d "
            "milliseconds. REACHED its max allowed time of %i milliseconds, STOPPING ROS...",
            jointInfo.name.c_str(), joint_pos_rad, joint_pos_iu, abs_encoder->getLowerErrorSoftLimitIU(),
            abs_encoder->getUpperErrorSoftLimitIU(), time_in_error_limits.count(),
            jointInfo.limit.msec_until_error_when_in_error_soft_limits.count());
        return true;
    }
    RCLCPP_WARN_THROTTLE((*logger_), clock_, jointInfo.limit.soft_error_limit_warning_throttle_msec,
        "Joint %s (%g rad; %i IU) is outside its soft ERROR limits [%i, %i], for %d milliseconds",
        jointInfo.name.c_str(), joint_pos_rad, joint_pos_iu, abs_encoder->getLowerErrorSoftLimitIU(),
        abs_encoder->getUpperErrorSoftLimitIU(), time_in_error_limits.count());

    // HARD limit check.
    if (jointInfo.limit.stop_when_outside_hard_limits && !jointInfo.joint.isWithinHardLimits()) {
        RCLCPP_FATAL((*logger_),
            "Joint %s IS OUTSIDE ITS HARD LIMIT STOPPING. "
            "\n\tposition: %g rad; %i IU."
            "\n\thard limit: [%i, %i]"
            "\n\thard limit rad: [%g, %g]",
            jointInfo.name.c_str(), joint_pos_rad, joint_pos_iu, abs_encoder->getLowerHardLimitIU(),
            abs_encoder->getUpperHardLimitIU(), abs_encoder->positionIUToRadians(abs_encoder->getLowerHardLimitIU()),
            abs_encoder->positionIUToRadians(abs_encoder->getUpperHardLimitIU()));
        return true;
    }
    return false;
}

std::unique_ptr<march::MarchRobot> MarchExoSystemInterface::load_march_hardware(
    const hardware_interface::HardwareInfo& info) const
{
    auto pos_iterator = info.hardware_parameters.find("robot");
    if (pos_iterator == info.hardware_parameters.end()) {
        throw std::invalid_argument("Hardware Parameter 'robot' not specified. Check under your "
                                    "'<hardware>' tag in the 'control/xacro/ros2_control.xacro' file.");
    }
    string robot_config_file_path = ament_index_cpp::get_package_share_directory("march_hardware_builder")
        + PATH_SEPARATOR + "robots" + PATH_SEPARATOR + pos_iterator->second + ".yaml";

    RCLCPP_INFO((*logger_), "Robot config file path: %s", robot_config_file_path.c_str());

    HardwareBuilder hw_builder { /*yaml_path=*/robot_config_file_path };

    std::vector<std::string> joint_names;
    joint_names.reserve(info.joints.size());
    for (const auto& joint : info.joints) {
        joint_names.emplace_back(joint.name.c_str());
    }

    return hw_builder.createMarchRobot(/*active_joint_names=*/joint_names);
}

bool MarchExoSystemInterface::has_correct_actuation_mode(march::Joint& joint) const
{
    const auto& actuation_mode = joint.getMotorController()->getActuationMode();
    if (actuation_mode != march::ActuationMode::torque) {
        RCLCPP_FATAL((*logger_),
            "Actuation mode for joint %s is not torque, but is %s.\n "
            "Check your `march_hardware_builder/robots/... .yaml`.",
            joint.getName().c_str(), actuation_mode.toString().c_str());
        return false;
    }
    return true;
}

} // namespace march_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(march_hardware_interface::MarchExoSystemInterface, hardware_interface::SystemInterface)
