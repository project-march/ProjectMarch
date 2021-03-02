#ifndef MARCH_YAML_UTILITIES_H
#define MARCH_YAML_UTILITIES_H
#include "yaml-cpp/yaml.h"

// Grabs a parameter from a YAML::Node and throws a clear warning if the requested parameter does not exist
template <class T>
void grabParameter(YAML::Node const yaml_node, std::string const parameter_name, T& parameter);

#endif //MARCH_YAML_UTILITIES_H
