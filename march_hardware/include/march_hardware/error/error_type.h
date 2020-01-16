// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_ERROR_TYPE_H
#define MARCH_HARDWARE_ERROR_TYPE_H
#include <ostream>

namespace march
{
namespace error
{
enum class ErrorType
{
  UNKNOWN = 999,
};

const char* getErrorDescription(ErrorType type)
{
  switch (type)
  {
    case ErrorType::UNKNOWN:
      return "Unknown error occured. Please create/use a documented error";
  }
}

std::ostream& operator<<(std::ostream& s, ErrorType type)
{
  s << "E" << static_cast<int>(type) << ": " << getErrorDescription(type);
  return s;
}
}  // namespace error
}  // namespace march

#endif  // MARCH_HARDWARE_ERROR_TYPE_H
