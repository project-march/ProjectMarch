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
    case ErrorType::INVALID_ACTUATE_POSITION:
      return "Invalid IU position command";
    case ErrorType::ENCODER_RESET:
      return "An encoder has reset and reads an incorrect value";
    case ErrorType::OUTSIDE_HARD_LIMITS:
      return "A joint is outside its defined hard limits";
    case ErrorType::TARGET_EXCEEDS_MAX_DIFFERENCE:
      return "The target position exceeds the max allowed difference from the current position";
    case ErrorType::TARGET_TORQUE_EXCEEDS_MAX_TORQUE:
      return "The target torque exceeds the max allowed torque";
    case ErrorType::PDO_OBJECT_NOT_DEFINED:
      return "The to be added PDO object was not defined";
    case ErrorType::PDO_REGISTER_OVERFLOW:
      return "The PDO map could not fit within the registers";
    case ErrorType::WRITING_INITIAL_SETTINGS_FAILED:
      return "Failed to write initial settings to slave required for operation";
    case ErrorType::NO_SOCKET_CONNECTION:
      return "Failed to connect to socket";
    case ErrorType::NOT_ALL_SLAVES_FOUND:
      return "Not all configured slaves were able to be found";
    case ErrorType::FAILED_TO_REACH_OPERATIONAL_STATE:
      return "At least one slave failed to reach ethercat operational state";
    case ErrorType::INVALID_ENCODER_RESOLUTION:
      return "The encoder resolution is outside the allowed range";
    case ErrorType::INVALID_RANGE_OF_MOTION:
      return "The lower and upper limits of an encoder are conflicting";
    default:
      return "Unknown error occurred. Please create/use a documented error";
  }
}

std::ostream& operator<<(std::ostream& s, ErrorType type)
{
  s << "E" << static_cast<int>(type) << ": " << getErrorDescription(type);
  return s;
}
}  // namespace error
}  // namespace march
