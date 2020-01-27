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
  INVALID_ACTUATION_MODE,
  INVALID_ACTUATE_POSITION,
  ENCODER_RESET,
  OUTSIDE_HARD_LIMITS,
  PDO_OBJECT_NOT_DEFINED,
  PDO_REGISTER_OVERFLOW,
  UNKNOWN = 999,
};

const char* getErrorDescription(ErrorType type);

std::ostream& operator<<(std::ostream& s, ErrorType type);
}  // namespace error
}  // namespace march

#endif  // MARCH_HARDWARE_ERROR_TYPE_H
