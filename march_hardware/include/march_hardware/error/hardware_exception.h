// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#define MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#include "error_type.h"

#include <exception>
#include <ostream>
#include <vector>

namespace march
{
namespace error
{
class HardwareException : public std::exception
{
public:
  HardwareException() = default;
  explicit HardwareException(ErrorType type) : type_(type)
  {
  }
  HardwareException(ErrorType type, std::string message) : type_(type), message_(std::move(message))
  {
  }

  template <typename... Args>
  HardwareException(ErrorType type, const std::string& format, Args... args) : type_(type)
  {
    const size_t size = std::snprintf(nullptr, 0, format.c_str(), args...);
    std::vector<char> buffer(size + 1);  // note +1 for null terminator
    std::snprintf(&buffer[0], buffer.size(), format.c_str(), args...);

    this->message_ = std::string(buffer.data(), size);
  }

  const char* what() const noexcept override
  {
    return "hardware exception";
  }

  ErrorType type() const noexcept
  {
    return this->type_;
  }

  friend std::ostream& operator<<(std::ostream& s, const HardwareException& e)
  {
    s << e.type_;
    if (!e.message_.empty())
    {
      s << std::endl << e.message_;
    }
    return s;
  }

private:
  const ErrorType type_ = ErrorType::UNKNOWN;
  std::string message_;
};

}  // namespace error
}  // namespace march

#endif  // MARCH_HARDWARE_HARDWARE_EXCEPTION_H
