// Copyright 2020 Project March.
#include "march_hardware/error/motor_controller_error.h"

namespace march
{
namespace error
{
const char* IMOTIONCUBE_MOTION_ERRORS[IMOTIONCUBE_MOTION_ERRORS_SIZE] = {
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

const char* IMOTIONCUBE_DETAILED_MOTION_ERRORS[IMOTIONCUBE_DETAILED_MOTION_ERRORS_SIZE] = {
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

const char* IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS[IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR_SIZE] = {
  "BiSS data CRC error. ",
  "BiSS data warning bit is set. ",
  "BiSS data error bit is set. ",
  "BiSS sensor missing. ",
  "Absolute Encoder Interface (AEI) interface error. ",
  "Hall sensor missing. ",
  "Position wraparound. The position 2^31 was exceeded. ",
};

const char* ODRIVE_AXIS_ERRORS[ODRIVE_AXIS_ERRORS_SIZE] = {
    "Invalid state. ",
    "Watchdog timer expired. ",
    "Min endstop pressed. ",
    "Max endstop pressed. ",
    "Estop requested. ",
    "Homing without endstop. ",
    "Over temperature. ",
    "Invalid encoder chosen. ",
};

const char* ODRIVE_MOTOR_ERRORS[ODRIVE_MOTOR_ERRORS_SIZE] = {
    "Phase resistance out of range. ",
    "Phase inductance out of range. ",
    "DRV fault. ",
    "Control deadline missed. ",
    "Modulation magnitude. ",
    "Current sense saturation. ",
    "Current limit violation. ",
    "Modulation is NaN. ",
    "Motor thermistor over temperature. ",
    "FET thermistor over temperature. ",
    "Timer update missed. ",
    "Current measurement unavailable. ",
    "Controller failed. ",
    "I bus out of range. ",
    "Brake resistor disarmed. ",
    "System level. ",
    "Bad timing. ",
    "Unknown phase estimate. ",
    "Unknown phase velocity. ",
    "Unknown torque. ",
    "Unknown current command. ",
    "Unknown current measurement. ",
    "Unknown vbus voltage. ",
    "Unknown voltage command. ",
    "Unknown gains. ",
    "Controller initializing. ",
};

const char* ODRIVE_ENCODER_ERRORS[ODRIVE_ENCODER_ERRORS_SIZE] = {
    "Unstable gain. ",
    "CPR polepairs mismatch. ",
    "No response. ",
    "Unsupported encoder mode. ",
    "Illegal hall state. ",
    "Index not found yet. ",
    "ABS SPI timeout. ",
    "ABS SPI communication failure. ",
    "ABS SPI not ready. ",
    "Insufficient resources. ",
    "Infeasible IO num. ",
};

const char* ODRIVE_ENCODER_MANAGER_ERRORS[ODRIVE_ENCODER_MANAGER_ERRORS_SIZE] = {
  "Error. "
};

const char* ODRIVE_CONTROLLER_ERRORS[ODRIVE_CONTROLLER_ERRORS_SIZE] = {
    "Overspeed. ",
    "Invalid input mode. ",
    "Unstable gain. ",
    "Invalid mirror axis. ",
    "Invalid load encoder. ",
    "Invalid estimate. ",
};

void addErrorToDescription(size_t index, ErrorRegister error_register, std::string &description)
{
  switch (error_register)
  {
    case ErrorRegister::IMOTIONCUBE_MOTION_ERROR:
      description += IMOTIONCUBE_MOTION_ERRORS[index];
      break;
    case ErrorRegister::IMOTIONCUBE_DETAILED_MOTION_ERROR:
      description += IMOTIONCUBE_DETAILED_MOTION_ERRORS[index];
      break;
    case ErrorRegister::IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR:
      description += IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS[index];
      break;
    case ErrorRegister::ODRIVE_AXIS_ERROR:
      description += ODRIVE_AXIS_ERRORS[index];
      break;
    case ErrorRegister::ODRIVE_MOTOR_ERROR:
      description += ODRIVE_MOTOR_ERRORS[index];
      break;
    case ErrorRegister::ODRIVE_ENCODER_ERROR:
      description += ODRIVE_ENCODER_ERRORS[index];
      break;
    case ErrorRegister::ODRIVE_ENCODER_MANAGER_ERROR:
      description += ODRIVE_ENCODER_MANAGER_ERRORS[index];
      break;
    case ErrorRegister::ODRIVE_CONTROLLER_ERROR:
      description += ODRIVE_CONTROLLER_ERRORS[index];
      break;
    default:
      break;
  }
}

std::string parseError(uint16_t error, ErrorRegister error_register)
{
  std::string description;
  const std::bitset<16> bitset(error);

  for (size_t i = 0; i < 16; i++)
  {
    if (bitset.test(i))
    {
      addErrorToDescription(i, error_register, description);
    }
  }
  return description;
}

std::string parseError(uint32_t error, ErrorRegister error_register)
{
  std::string description;
  const std::bitset<32> bitset(error);

  for (size_t i = 0; i < 32; i++)
  {
    if (bitset.test(i))
    {
      addErrorToDescription(i, error_register, description);
    }
  }
  return description;
}
}  // namespace error
}  // namespace march
