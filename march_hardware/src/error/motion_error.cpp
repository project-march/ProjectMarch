// Copyright 2020 Project March.
#include <march_hardware/error/motion_error.h>

namespace march
{
namespace error
{
const char* MOTION_ERRORS[MOTION_ERRORS_SIZE] = {
    "EtherCAT communication error. ",
    "Short-circuit. ",
    "Invalid setup (EEPROM) data. ",
    "Control error (position/speed error too big). ",
    "Communication error. ",
    "Motor position wraps around. ",
    "Positive limit switch. ",
    "Negative limit switch. ",
    "Over-current. ",
    "I2T protection. ",
    "Over-temperature motor. ",
    "Over-temperature drive. ",
    "Over-voltage. ",
    "Under-voltage. ",
    "Command error. ",
    "Drive disabled (Emergency button connector not shorted). ",
};

const char* DETAILED_MOTION_ERRORS[DETAILED_MOTION_ERRORS_SIZE] = {
    "TML stack overflow. ",
    "TML stack underflow. ",
    "Homing not available. ",
    "Function not available. ",
    "UPD ignored. ",
    "Cancelable call ignored. ",
    "Positive software limit switch is active. ",
    "Negative software limit switch is active. ",
    "Invalid S-curve profile. ",
};

std::string parseMotionError(uint16_t motion_error)
{
  std::string description;
  const std::bitset<16> bitset(motion_error);
  for (size_t i = 0; i < MOTION_ERRORS_SIZE; i++)
  {
    if (bitset.test(i))
    {
      description += MOTION_ERRORS[i];
    }
  }
  return description;
}

std::string parseDetailedError(uint16_t detailed_error)
{
  std::string description;
  const std::bitset<16> bitset(detailed_error);
  for (size_t i = 0; i < DETAILED_MOTION_ERRORS_SIZE; i++)
  {
    if (bitset.test(i))
    {
      description += DETAILED_MOTION_ERRORS[i];
    }
  }
  return description;
}
}
}
