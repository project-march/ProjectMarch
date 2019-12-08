// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#define MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#include <exception>
#include <string>

#include <ros/ros.h>

class HardwareConfigException : public std::exception
{
public:
  std::string msg;

  HardwareConfigException() : std::exception(), msg("Invalid hardware configuration")
  {
  }

  explicit HardwareConfigException(std::string msg) : std::exception(), msg(msg)
  {
  }

  const char* what() const throw()
  {
    return msg.c_str();
  }
};

class MissingKeyException : public HardwareConfigException
{
public:
  std::string key_name;
  std::string object_name;

  explicit MissingKeyException(std::string key_name, std::string object_name)
    : HardwareConfigException(), key_name(key_name), object_name(object_name)
  {
    std::ostringstream ss;
    ss << "Missing key '" << key_name << "' while creating object '" << object_name << "'";
    this->msg = ss.str();
  }
};
#endif  // MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
