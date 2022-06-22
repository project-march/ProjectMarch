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

using namespace std;

namespace march_hardware_interface_util {

string string_array_to_string(const vector<string>& string_array)
{
    string ret_string = "[ ";
    for (const string& s : string_array) {
        ret_string.append(s + ", ");
    }
    ret_string += "]";
    return ret_string;
}

string interface_info_to_string(const vector<hardware_interface::InterfaceInfo>& interfaces)
{
    string ret_string;
    for (const hardware_interface::InterfaceInfo& interface : interfaces) {
        ret_string.append(interface.name);
    }
    return ret_string;
}

/// Returns a sorted list of the interface names.
vector<string> get_sorted_interface_names(const vector<hardware_interface::InterfaceInfo>& interfaces)
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
bool has_required_interfaces(const vector<hardware_interface::InterfaceInfo>& interfaces,
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
bool joints_have_interface_types(const vector<hardware_interface::ComponentInfo>& joints,
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

} // namespace march_hardware_interface_util

#endif // MARCH_HARDWARE_INTERFACE__HWI_UTIL_H
