// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#define MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#include <sstream>
#include <string>

#include <march_hardware/error/error_type.h>
#include <march_hardware/error/hardware_exception.h>

class MissingKeyException : public march::error::HardwareException
{
public:
  MissingKeyException(const std::string& key_name, const std::string& object_name)
    : march::error::HardwareException(march::error::ErrorType::MISSING_REQUIRED_KEY)
  {
    std::ostringstream ss;
    ss << "Missing required key '" << key_name << "' while creating object '" << object_name << "'";
    this->description_ = this->createDescription(ss.str());
  }
};
#endif  // MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
