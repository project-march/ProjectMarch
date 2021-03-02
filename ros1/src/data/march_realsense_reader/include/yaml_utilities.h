#ifndef MARCH_YAML_UTILITIES_H
#define MARCH_YAML_UTILITIES_H
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

// Grabs a parameter from a YAML::Node and throws a clear warning if the requested parameter does not exist
template <typename T>
void grabParameter(YAML::Node const yaml_node, std::string const parameter_name, T parameter)
{
  if (YAML::Node raw_parameter = yaml_node[parameter_name])
  {
    parameter = raw_parameter.as<T>();
  }
  else
  {
    ROS_ERROR_STREAM("parameter not found in the given YAML::node. Parameter name is " << parameter_name);
  }
}

#endif //MARCH_YAML_UTILITIES_H
