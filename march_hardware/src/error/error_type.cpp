// Copyright 2020 Project March.
#include "march_hardware/error/error_type.h"

namespace march
{
namespace error
{
const char* getErrorDescription(ErrorType type)
{
  switch (type)
  {
    case ErrorType::INVALID_ACTUATION_MODE:
      return "An invalid actuation mode was used to perform an action";
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
