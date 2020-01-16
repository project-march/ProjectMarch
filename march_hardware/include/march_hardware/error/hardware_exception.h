// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#define MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#include "error_type.h"

#include <exception>
#include <ostream>

namespace march
{
namespace error
{
class HardwareException : public std::exception
{
public:
  HardwareException() = default;
  HardwareException(ErrorType type) : type_(type)
  {
  }
  HardwareException(ErrorType type, std::string message) : type_(type), message_(std::move(message))
  {
  }

  const char* what() const noexcept override
  {
    return "hardware exception";
  }

  friend std::ostream& operator<<(std::ostream& s, const HardwareException& e)
  {
    s << e.type_ << std::endl << e.message_;
    return s;
  }

private:
  const ErrorType type_ = ErrorType::UNKNOWN;
  const std::string message_;
};

}  // namespace error
}  // namespace march

#endif  // MARCH_HARDWARE_HARDWARE_EXCEPTION_H
