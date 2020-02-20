// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#define MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#include "error_type.h"

#include <exception>
#include <ostream>
#include <string>
#include <sstream>
#include <vector>

namespace march
{
namespace error
{
class HardwareException : public std::exception
{
public:
  explicit HardwareException(ErrorType type) : HardwareException(type, "")
  {
  }

  HardwareException(ErrorType type, const std::string& message)
    : type_(type), description_(this->createDescription(message))
  {
  }

  template <typename... Args>
  HardwareException(ErrorType type, const std::string& format, Args... args) : type_(type)
  {
    const size_t size = std::snprintf(nullptr, 0, format.c_str(), args...);
    std::vector<char> buffer(size + 1);  // note +1 for null terminator
    std::snprintf(&buffer[0], buffer.size(), format.c_str(), args...);

    this->description_ = this->createDescription(std::string(buffer.data(), size));
  }

  const char* what() const noexcept override
  {
    return this->description_.c_str();
  }

  ErrorType type() const noexcept
  {
    return this->type_;
  }

  friend std::ostream& operator<<(std::ostream& s, const HardwareException& e)
  {
    s << e.description_;
    return s;
  }

private:
  std::string createDescription(const std::string& message)
  {
    std::stringstream ss;
    ss << this->type_;
    if (!message.empty())
    {
      ss << std::endl;
      ss << message;
    }
    return ss.str();
  }

  const ErrorType type_ = ErrorType::UNKNOWN;
  std::string description_;
};

}  // namespace error
}  // namespace march

#endif  // MARCH_HARDWARE_HARDWARE_EXCEPTION_H
