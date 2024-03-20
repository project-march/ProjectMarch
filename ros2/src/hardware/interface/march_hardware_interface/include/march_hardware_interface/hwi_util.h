//
// Created by george on 14-6-22.
//

#ifndef MARCH_HARDWARE_INTERFACE__HWI_UTIL_H
#define MARCH_HARDWARE_INTERFACE__HWI_UTIL_H

#include "march_hardware/march_robot.h"
#include "march_utility/logger_colors.hpp"
#include <algorithm>
#include <csignal>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using namespace std;

namespace march_hardware_interface_util {

inline string string_array_to_string(const vector<string>& string_array)
{
    string ret_string = "[ ";
    for (const string& s : string_array) {
        ret_string.append(s + ", ");
    }
    ret_string += "]";
    return ret_string;
}

inline string interface_info_to_string(const vector<hardware_interface::InterfaceInfo>& interfaces)
{
    string ret_string;
    for (const hardware_interface::InterfaceInfo& interface : interfaces) {
        ret_string.append(interface.name);
    }
    return ret_string;
}

/// Returns a sorted list of the interface names.
inline vector<string> get_sorted_interface_names(const vector<hardware_interface::InterfaceInfo>& interfaces)
{
    vector<string> sorted_interface_names { interfaces.size() };
    transform(interfaces.begin(), interfaces.end(), sorted_interface_names.begin(),
        [](const hardware_interface::InterfaceInfo& interface) {
            return interface.name;
        });
    sort(sorted_interface_names.begin(), sorted_interface_names.end());
    return sorted_interface_names;
}

///// Checks if the interface names are the same as the required ones.
inline bool has_required_interfaces(const vector<hardware_interface::InterfaceInfo>& interfaces,
    vector<string> required_interface_names, const string& joint_name, const string& interface_type,
    const rclcpp::Logger& logger)
{
    sort(required_interface_names.begin(), required_interface_names.end());
    vector<string> interface_names = get_sorted_interface_names(interfaces);
    if (interface_names != required_interface_names) {
        RCLCPP_FATAL(logger,
            "Joint '%s' had %s %s interfaces, but should only have %s %s interfaces. \n"
            "Check your urdf for the interfaces defined in element '<joint name=%s>' "
            "which is an element of '<ros2_control ...>'.",
            joint_name.c_str(), string_array_to_string(interface_names).c_str(), interface_type.c_str(),
            string_array_to_string(required_interface_names).c_str(), interface_type.c_str(), joint_name.c_str());
        return false;
    }
    return true;
}

/// Check if the joint has the correct state & command interface.
inline bool joints_have_interface_types(const vector<hardware_interface::ComponentInfo>& joints,
    const vector<string>& required_command_interfaces, const vector<string>& required_state_interfaces,
    const rclcpp::Logger& logger)
{
    for (const hardware_interface::ComponentInfo& joint : joints) {
        if (!has_required_interfaces(
                joint.command_interfaces, required_command_interfaces, joint.name, "command", logger)) {
            return false;
        }
        if (!has_required_interfaces(joint.state_interfaces, required_state_interfaces, joint.name, "state", logger)) {
            return false;
        }
    }

    return true;
}

inline string get_parameter(
    const hardware_interface::ComponentInfo& component, const string& parameter, const string& default_value)
{
    const auto& params = component.parameters;
    auto pos_iterator = params.find(parameter);
    if (pos_iterator == params.end()) {
        return default_value;
    } else {
        return pos_iterator->second;
    }
}

/// To switch path separators when on windows.
inline static const char PATH_SEPARATOR =
#ifdef _WIN32
    '\\';
#else
    '/';
#endif

inline string joint_vector_to_string(std::vector<march::Joint*>& joints)
{
    std::stringstream joint_names_ss;
    joint_names_ss << "[ ";
    for (ulong i = 0; i < joints.size() - 1; i++) {
        joint_names_ss << joints[i]->getName().c_str() << ", ";
    }
    joint_names_ss << joints[joints.size() - 1]->getName().c_str() << " ]";
    return joint_names_ss.str();
}

/** \brief Executes a given function on joint until successful with a set amount of maximum tries.
 *
 * @param function_goal The goal of the function. This is logged if it did not succeed in the maximum amount of tries.
 * @param function The function that should be executed on the joint until successful. The success of the function
 *                  is defined by the return boolean of the function.
 * @param logger This is used to log with verbosity 'ERROR' if the function did not succeed for every joint.
 * @param robot The robot to get the march::joints from.
 * @param function_when_timeout The function that is executed on all joints that did not succeed.
 * @param sleep_between_tries The time between retries.
 * @param maximum_tries The maximum number of retries.
 * \throws march::error::HardwareException if it did not succeed to execute for every joint in the given amount of
 * tries.
 */
inline void repeat_function_on_joints_until_timeout(const string& function_goal,
    const function<bool(march::Joint&)>& function, const rclcpp::Logger& logger, std::vector<march::Joint*>& joints,
    const optional<std::function<void(march::Joint&)>>& function_when_timeout = nullopt,
    const chrono::nanoseconds sleep_between_tries = std::chrono::seconds(2), const unsigned maximum_tries = 12)
{
    // RCLCPP_INFO((logger), "repeat_function_on_joints_until_timeout");
    vector<bool> is_ok;
    unsigned int amount_ok = 0;
    unsigned int amount_of_joints = joints.size();
    RCLCPP_INFO(logger, "There are %d joints to wait on", amount_of_joints);
    if (amount_of_joints != 0) {
        RCLCPP_INFO(logger, "There are %d joints to wait on", amount_of_joints);
        is_ok.resize(amount_of_joints, /*__x=*/false);
    }

    // RCLCPP_INFO(logger, "Trying to perform %s'%s'%s on joints: '%s' in %i tries. %sMight take %f seconds.",
    //     LColor::BOLD, function_goal.c_str(), LColor::END, joint_vector_to_string(joints).c_str(), maximum_tries,
    //     LColor::WARNING, sleep_between_tries.count() / 1000000000.0 * maximum_tries);
    unsigned int num_tries = 0;
    for (; num_tries < maximum_tries; num_tries++) {
        for (unsigned int i = 0; i < amount_of_joints; i++) {
            if (!is_ok.at(i) && function(/*input_to_given_function=*/*joints[i])) {
                amount_ok++;
                is_ok.at(i) = true;
                if (amount_ok == amount_of_joints) {
                    return;
                }
            }
        }
        rclcpp::sleep_for(sleep_between_tries);
    }

    if (amount_ok != amount_of_joints) {
        for (unsigned int i = 0; i < amount_of_joints; i++) {
            if (!is_ok.at(i)) {
                RCLCPP_ERROR(logger, "Couldn't perform '%s' on joint '%s' in %i tries.", function_goal.c_str(),
                    joints[i]->getName().c_str(), num_tries);
            }
        }
        if (function_when_timeout.has_value()) {
            auto const& callable = function_when_timeout.value();
            for (unsigned int i = 0; i < amount_of_joints; i++) {
                if (!is_ok.at(i)) {
                    callable(*joints[i]);
                }
            }
        }
        throw march::error::HardwareException(march::error::ErrorType::BUSY_WAITING_FUNCTION_MAXIMUM_TRIES_REACHED);
    }
}

inline void call_function_and_wait_on_joints(const string& function_goal,
    const function<const chrono::nanoseconds(march::Joint&)>& function, const rclcpp::Logger& logger,
    std::vector<march::Joint*>& joints)
{
    RCLCPP_INFO(logger, "Performing %s'%s'%s on joints: '%s'.", LColor::BOLD, function_goal.c_str(), LColor::END,
        joint_vector_to_string(joints).c_str());
    chrono::nanoseconds max_sleep { 0 };
    for (auto joint : joints) {
        max_sleep = max(max_sleep, function(*joint));
    }
    if (max_sleep.count() > 0) {
        RCLCPP_INFO(logger, "%s %s Successful after %.4f seconds.", LColor::BLUE, function_goal.c_str(),
            max_sleep.count(), max_sleep.count() / 1000000000.0);
    }
    rclcpp::sleep_for(max_sleep);
}

inline bool is_ethercat_alive(const std::exception_ptr& exceptionPtr, const rclcpp::Logger& logger)
{
    if (exceptionPtr) {
        try {
            std::rethrow_exception(exceptionPtr);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(logger, "Ethercat threw exception: \n\t%s", e.what());
        }
        return false;
    }
    return true;
}

// Original: check whether the motor controller is in a valid state > state 8 (closed loop control).
// Currently: check whether the motor controller has an error.
// TODO: check which one is better > rename function in the cleanup if current implementation is used.
inline bool is_motor_controller_in_a_valid_state(march::Joint& joint, const rclcpp::Logger& logger)
{
    auto motor_controller_state = joint.getMotorController()->getState();
    if (motor_controller_state->hasError()) {
        RCLCPP_ERROR(logger, "MotorController of joint %s is in fault state %s.\n Error Status: \n%s",
            joint.getName().c_str(), motor_controller_state->getOperationalState().c_str(),
            motor_controller_state->getErrorStatus().c_str());
        return false;
    }
    return true;
}

/** \brief This is done because the ros2 code doesn't yet correctly go to the teardown state.
 * \note This only works if there is one instance alive of this object.
 * \attention Because this is global, this code can only be ran once within this namespace.
 */
inline hardware_interface::BaseInterface<hardware_interface::SystemInterface>* glob_instance;
inline void teardown_state_cb(int signum)
{
    glob_instance->stop();
    exit(signum);
}

/**
 * \brief This ensures that whenever a user interrupt or exception is thrown the hardware_interface still goes through
 * the `stop` state (aka the stop method).
 * @param instance The hardware interface instance that should go ot the stop state.
 */
inline void go_to_stop_state_on_crash(hardware_interface::BaseInterface<hardware_interface::SystemInterface>* instance)
{
    glob_instance = instance;
    signal(SIGINT, teardown_state_cb); // For user interrupt.
    signal(SIGTERM, teardown_state_cb); // For termination request, sent to the program.
    signal(SIGABRT, teardown_state_cb); // For abnormal termination condition, (e.g. thrown exceptions).
}

} // namespace march_hardware_interface_util

#endif // MARCH_HARDWARE_INTERFACE__HWI_UTIL_H
