#ifndef MARCH_YAML_UTILITIES_H
#define MARCH_YAML_UTILITIES_H
#include "yaml-cpp/yaml.h"
#include <optional>
#include <ros/ros.h>
#include <string>
#include <typeinfo>

namespace yaml_utilities {
// Grab a parameter from a YAML::Node and throws a clear warning if the
// requested parameter does not exist
template <typename T>
std::optional<T> grabParameter(
    YAML::Node const& yaml_node, std::string const& parameter_name)
{
    T parameter;
    if (YAML::Node raw_parameter = yaml_node[parameter_name]) {
        parameter = raw_parameter.as<T>();
        return parameter;
    } else {
        ROS_ERROR_STREAM(
            "Parameter not found in the given YAML::node. Parameter name is "
            << parameter_name);
        return std::nullopt;
    }
}
} // namespace yaml_utilities

#endif // MARCH_YAML_UTILITIES_H
