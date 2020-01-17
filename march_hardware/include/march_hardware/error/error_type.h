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
  PDO_OBJECT_NOT_DEFINED,
  PDO_REGISTER_OVERFLOW,
  UNKNOWN = 999,
};

const char* getErrorDescription(ErrorType type)
{
  switch (type)
  {
    case ErrorType::PDO_OBJECT_NOT_DEFINED:
      return "The to be added PDO object was not defined";
    case ErrorType::PDO_REGISTER_OVERFLOW:
      return "The PDO map could not fit within the registers";
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
