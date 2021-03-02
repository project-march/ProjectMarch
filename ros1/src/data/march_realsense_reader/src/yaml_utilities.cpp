#include "yaml_utilities.h"
#include "yaml-cpp/yaml.h"

// Grabs a parameter from a YAML::Node and throws a clear warning if the requested parameter does not exist
template <typename T>
void grabParametergits(YAML::Node const yaml_node, std::string const parameter_name, T parameter)
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

