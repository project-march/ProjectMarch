//
// Created by george on 14-6-22.
//

#ifndef MARCH_HARDWARE_INTERFACE__HWI_UTIL_H
#define MARCH_HARDWARE_INTERFACE__HWI_UTIL_H

#include <algorithm>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "march_hardware/march_robot.h"

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
    const vector<string>& required_interface_names, const string& joint_name, const string& interface_type,
    const rclcpp::Logger& logger)
{
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

inline string get_parameter(const hardware_interface::ComponentInfo& component, const string& parameter,
                             const string& default_value ) {
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

///** \brief Executes a given function on joint until successful with a set amount of maximum tries.
// *
// * @param function_goal The goal of the function. This is logged if it did not succeed in the maximum amount of tries.
// * @param function The function that should be executed on the joint until successful. The success of the function
// *                  is defined by the return boolean of the function.
// * @param logger This is used to log with verbosity 'ERROR' if the function did not succeed for every joint.
// * @param robot The robot to get the march::joints from.
// * @param function_when_timeout The function that is executed on all joints that did not succeed.
// * @param sleep_between_tries The time between retries.
// * @param maximum_tries The maximum number of retries.
// * \throws march::error::HardwareException if it did not succeed to execute for every joint in the given amount of tries.
// */
inline void repeat_function_on_joints_until_timeout(const string &function_goal,
                                                    const function<bool(march::Joint &)> &function,
                                                    const rclcpp::Logger &logger,
                                                    march::MarchRobot *robot,
                                                    const optional<std::function<void(march::Joint &)>> &
                                                        function_when_timeout = nullopt,
                                                    const chrono::nanoseconds sleep_between_tries
                                                        = std::chrono::nanoseconds(1),
                                                    const unsigned maximum_tries  = 10) {
    vector<bool> is_ok;
    unsigned int amount_ok = 0;
    unsigned int amount_of_joints = robot->size();
    is_ok.resize(amount_of_joints, false);

    unsigned int num_tries = 0;
    for (; num_tries < maximum_tries; num_tries++) {
        for (unsigned int i = 0; i < amount_of_joints; i++) {
            if (!is_ok.at(i) && function(/*input_to_given_function=*/robot->getJoint(i))) {
                amount_ok++;
                is_ok.at(i) = true;
                if (amount_ok == amount_of_joints) { break; }
            }
            rclcpp::sleep_for(sleep_between_tries);
        }

        if (amount_ok != amount_of_joints) {
            for (unsigned int i = 0; i < amount_of_joints; i++) {
                if (!is_ok.at(i)) {
                    RCLCPP_ERROR(logger, "Couldn't perform %s on joint '%s' in %i tries.",
                                 function_goal.c_str(), robot->getJoint(i).getName().c_str(), num_tries);
                }
            }
            if (function_when_timeout.has_value()) {
                auto const &callable = function_when_timeout.value();
                for (unsigned int i = 0; i < amount_of_joints; i++) {
                    if (!is_ok.at(i)) {
                        callable(robot->getJoint(i));
                    }
                }
            }
            throw march::error::HardwareException(
                    march::error::ErrorType::BUSY_WAITING_FUNCTION_MAXIMUM_TRIES_REACHED);
        }
    }
}


} // namespace march_hardware_interface_util

#endif // MARCH_HARDWARE_INTERFACE__HWI_UTIL_H
