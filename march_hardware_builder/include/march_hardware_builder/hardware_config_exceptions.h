// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#define MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#include <exception>
#include <sstream>
#include <string>

class HardwareConfigException : public std::exception
{
public:
  HardwareConfigException() : msg("Invalid hardware configuration")
  {
  }

  explicit HardwareConfigException(std::string msg) : msg(std::move(msg))
  {
  }

  const char* what() const noexcept override
  {
    return msg.c_str();
  }

protected:
  std::string msg;
};

class MissingKeyException : public HardwareConfigException
{
public:
  MissingKeyException(std::string key_name, std::string object_name)
    : key_name(std::move(key_name)), object_name(std::move(object_name))
  {
    std::ostringstream ss;
    ss << "Missing required key '" << key_name << "' while creating object '" << object_name << "'";
    this->msg = ss.str();
  }

private:
  std::string key_name;
  std::string object_name;
};
#endif  // MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
