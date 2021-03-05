// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ODRIVE_STATE_H
#define MARCH_HARDWARE_ODRIVE_STATE_H

#include <string>
#include "march_hardware/motor_controller/motor_controller_state.h"

namespace march
{

template<class Value>
Value getNextValue(Value current)
{
  return static_cast<Value>(static_cast<uint32_t>(current) << 1);
}


template<class ErrorType, class ErrorValue>
std::string errorsToString(uint32_t error_status, std::string error_type, ErrorValue begin, ErrorValue end)
{
  std::ostringstream ss;
  ss << error_type << " errors: ";
  if (error_status == 0)
  {
    ss << "None";
  }
  else
  {
    ss << "[";
    for(auto possible_error = begin; possible_error <= end; possible_error = getNextValue(possible_error))
    {
      if ((error_status & possible_error) > 0)
      {
        ss << ErrorType::toString(possible_error) << ", ";
      }
    }
    ss << "]";
  }
  return ss.str();
}

class ODriveAxisState
{
public:
  enum Value: int8_t
  {
    UNDEFINED = 0,
    IDLE = 1,
    STARTUP_SEQUENCE = 2,
    FULL_CALIBRATION_SEQUENCE = 3,
    MOTOR_CALIBRATION = 4,
    SENSORLESS_CONTROL = 5,
    ENCODER_INDEX_SEARCH = 6,
    ENCODER_OFFSET_CALIBRATION = 7,
    CLOSED_LOOP_CONTROL = 8,
    LOCKIN_SPIN = 9,
    ENCODER_DIR_FIND = 10,
    HOMING = 11,
    ENCODER_HALL_POLARITY_CALIBRATION = 12,
    ENCODER_HALL_PHASE_CALIBRATION = 13
  };

  ODriveAxisState() : value_(UNDEFINED)
  {
  }

  ODriveAxisState(int8_t state) : value_(Value(state))
  {
  }

  ODriveAxisState(Value value) : value_(value)
  {
  }

  std::string toString()
  {
    return toString(value_);
  }

  static std::string toString(Value value)
  {
    switch (value)
    {
      case UNDEFINED:
        return "Undefined";
      case IDLE:
        return "Idle";
      case STARTUP_SEQUENCE:
        return "Startup sequence";
      case FULL_CALIBRATION_SEQUENCE:
        return "Full calibration sequence";
      case MOTOR_CALIBRATION:
        return "Motor calibration";
      case SENSORLESS_CONTROL:
        return "Sensorless control";
      case ENCODER_INDEX_SEARCH:
        return "Encoder index search";
      case ENCODER_OFFSET_CALIBRATION:
        return "Encoder offset calibration";
      case CLOSED_LOOP_CONTROL:
        return "Close loop control";
      case LOCKIN_SPIN:
        return "Lockin spin";
      case ENCODER_DIR_FIND:
        return "Encoder dir find";
      case HOMING:
        return "Homing";
      case ENCODER_HALL_POLARITY_CALIBRATION:
        return "Encoder hall polarity calibration";
      case ENCODER_HALL_PHASE_CALIBRATION:
        return "Encoder hall phase calibration";
      default:
        return "Unrecognized ODrive axis state";
    }
  }

  bool operator==(Value v) const
  {
    return value_ == v;
  }
  bool operator==(ODriveAxisState a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(ODriveAxisState a) const
  {
    return value_ != a.value_;
  }

  Value value_;
};

class ODriveAxisError
{
public:
  enum Value: uint32_t
  {
    NONE                       = 0x00000000,
    INVALID_STATE              = 0x00000001,
    WATCHDOG_TIMER_EXPIRED     = 0x00000800,
    MIN_ENDSTOP_PRESSED        = 0x00001000,
    MAX_ENDSTOP_PRESSED        = 0x00002000,
    ESTOP_REQUESTED            = 0x00004000,
    HOMING_WITHOUT_ENDSTOP     = 0x00020000,
    OVER_TEMP                  = 0x00040000,
    INVALID_ENCODER_CHOSEN     = 0x00080000,
  };

  ODriveAxisError() : value_(0)
  {
  }

  ODriveAxisError(uint32_t value) : value_(value)
  {
  }

  static std::string toString(Value value)
  {
    switch (value)
    {
      case NONE:
        return "None";
      case INVALID_STATE:
        return "Invalid state";
      case WATCHDOG_TIMER_EXPIRED:
        return "Watchdog timer expired";
      case MIN_ENDSTOP_PRESSED:
        return "Min endstop pressed";
      case MAX_ENDSTOP_PRESSED:
        return "Max endstop pressed";
      case ESTOP_REQUESTED:
        return "Estop requested";
      case HOMING_WITHOUT_ENDSTOP:
        return "Homing without endstop";
      case OVER_TEMP:
        return "Over temperature";
      case INVALID_ENCODER_CHOSEN:
        return "Invalid encoder chosen";
      default:
        return "Unknown";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveAxisError, Value>(value_, "Axis", INVALID_STATE, INVALID_ENCODER_CHOSEN);
  }

  bool operator==(uint32_t v) const
  {
    return value_ == v;
  }
  bool operator==(ODriveAxisError a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(ODriveAxisError a) const
  {
    return value_ != a.value_;
  }

  uint32_t value_;
};

class ODriveMotorError
{
public:
  enum Value: uint64_t
  {
    NONE                            = 0x00000000,
    PHASE_RESISTANCE_OUT_OF_RANGE   = 0x00000001,
    PHASE_INDUCTANCE_OUT_OF_RANGE   = 0x00000002,
    DRV_FAULT                       = 0x00000008,
    CONTROL_DEADLINE_MISSED         = 0x00000010,
    MODULATION_MAGNITUDE            = 0x00000080,
    CURRENT_SENSE_SATURATION        = 0x00000400,
    CURRENT_LIMIT_VIOLATION         = 0x00001000,
    MODULATION_IS_NAN               = 0x00010000,
    MOTOR_THERMISTOR_OVER_TEMP      = 0x00020000,
    FET_THERMISTOR_OVER_TEMP        = 0x00040000,
    TIMER_UPDATE_MISSED             = 0x00080000,
    CURRENT_MEASUREMENT_UNAVAILABLE = 0x00100000,
    CONTROLLER_FAILED               = 0x00200000,
    I_BUS_OUT_OF_RANGE              = 0x00400000,
    BRAKE_RESISTOR_DISARMED         = 0x00800000,
    SYSTEM_LEVEL                    = 0x01000000,
    BAD_TIMING                      = 0x02000000,
    UNKNOWN_PHASE_ESTIMATE          = 0x04000000,
    UNKNOWN_PHASE_VEL               = 0x08000000,
    UNKNOWN_TORQUE                  = 0x10000000,
    UNKNOWN_CURRENT_COMMAND         = 0x20000000,
    UNKNOWN_CURRENT_MEASUREMENT     = 0x40000000,
    UNKNOWN_VBUS_VOLTAGE            = 0x80000000,
    UNKNOWN_VOLTAGE_COMMAND         = 0x100000000,
    UNKNOWN_GAINS                   = 0x200000000,
    CONTROLLER_INITIALIZING         = 0x400000000,
  };

  ODriveMotorError() : value_(0)
  {
  }

  ODriveMotorError(uint64_t value) : value_(value)
  {
  }

  static std::string toString(Value value)
  {
    switch (value)
    {
      case NONE:
        return "None";
      case PHASE_RESISTANCE_OUT_OF_RANGE:
        return "Phase resistance out of range";
      case PHASE_INDUCTANCE_OUT_OF_RANGE:
        return "Phase inductance out of range";
      case DRV_FAULT:
        return "DRV fault";
      case CONTROL_DEADLINE_MISSED:
        return "Control deadline missed";
      case MODULATION_MAGNITUDE:
        return "Modulation magnitude";
      case CURRENT_SENSE_SATURATION:
        return "Current sense saturation";
      case CURRENT_LIMIT_VIOLATION:
        return "Current limit violation";
      case MODULATION_IS_NAN:
        return "Modulation is NaN";
      case MOTOR_THERMISTOR_OVER_TEMP:
        return "Motor thermistor over temperature";
      case FET_THERMISTOR_OVER_TEMP:
        return "FET thermistor over temperature";
      case TIMER_UPDATE_MISSED:
        return "Timer update missed";
      case CURRENT_MEASUREMENT_UNAVAILABLE:
        return "Current measurement unavailable";
      case CONTROLLER_FAILED:
        return "Controller failed";
      case I_BUS_OUT_OF_RANGE:
        return "I bus out of range";
      case BRAKE_RESISTOR_DISARMED:
        return "Brake resistor disarmed";
      case SYSTEM_LEVEL:
        return "System level";
      case BAD_TIMING:
        return "Bad timing";
      case UNKNOWN_PHASE_ESTIMATE:
        return "Unknown phase estimate";
      case UNKNOWN_PHASE_VEL:
        return "Unknown phase velocity";
      case UNKNOWN_TORQUE:
        return "Unknown torque";
      case UNKNOWN_CURRENT_COMMAND:
        return "Unknown current command";
      case UNKNOWN_CURRENT_MEASUREMENT:
        return "Unknown current measurement";
      case UNKNOWN_VBUS_VOLTAGE:
        return "Unknown vbus voltage";
      case UNKNOWN_VOLTAGE_COMMAND:
        return "Unknown voltage command";
      case UNKNOWN_GAINS:
        return "Unknown gains";
      case CONTROLLER_INITIALIZING:
        return "Controller initializing";
      default:
        return "Unknown motor error";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveMotorError, Value>(value_, "Motor", PHASE_RESISTANCE_OUT_OF_RANGE, CONTROLLER_INITIALIZING);
  }

  bool operator==(uint64_t v) const
  {
    return value_ == v;
  }
  bool operator==(ODriveMotorError a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(ODriveMotorError a) const
  {
    return value_ != a.value_;
  }

  uint64_t value_;
};

class ODriveEncoderManagerError
{
public:
  enum Value: uint32_t
  {
    NONE = 0,
    ERROR = 1
  };

  ODriveEncoderManagerError() : value_(0)
  {
  }

  ODriveEncoderManagerError(uint32_t value) : value_(value)
  {
  }

  static std::string toString(Value value)
  {
    switch (value)
    {
      case NONE:
        return "None";
      case ERROR:
        return "Error";
      default:
        return "Unknown encoder manager error";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveEncoderManagerError, Value>(value_, "Encoder Manager", ERROR, ERROR);
  }

  bool operator==(uint32_t v) const
  {
    return value_ == v;
  }
  bool operator==(ODriveEncoderManagerError a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(ODriveEncoderManagerError a) const
  {
    return value_ != a.value_;
  }

  uint32_t value_;
};

class ODriveEncoderError
{
public:
  enum Value: uint32_t
  {
    NONE                       = 0x00000000,
    UNSTABLE_GAIN              = 0x00000001,
    CPR_POLEPAIRS_MISMATCH     = 0x00000002,
    NO_RESPONSE                = 0x00000004,
    UNSUPPORTED_ENCODER_MODE   = 0x00000008,
    ILLEGAL_HALL_STATE         = 0x00000010,
    INDEX_NOT_FOUND_YET        = 0x00000020,
    ABS_SPI_TIMEOUT            = 0x00000040,
    ABS_SPI_COM_FAIL           = 0x00000080,
    ABS_SPI_NOT_READY          = 0x00000100,
    INSUFFICIENT_RESOURCES     = 0x00000200,
    INFEASIBLE_IO_NUM          = 0x00000400
  };

  ODriveEncoderError() : value_(0)
  {
  }

  ODriveEncoderError(uint32_t value) : value_(value)
  {
  }

  static std::string toString(Value value)
  {
    switch (value)
    {
      case NONE:
        return "None";
      case UNSTABLE_GAIN:
        return "Unstable gain";
      case CPR_POLEPAIRS_MISMATCH:
        return "CPR polepairs mismatch";
      case NO_RESPONSE:
        return "No response";
      case UNSUPPORTED_ENCODER_MODE:
        return "Unsupported encoder mode";
      case ILLEGAL_HALL_STATE:
        return "Illegal hall state";
      case INDEX_NOT_FOUND_YET:
        return "Index not found yet";
      case ABS_SPI_TIMEOUT:
        return "ABS SPI timeout";
      case ABS_SPI_COM_FAIL:
        return "ABS SPI communication failure";
      case ABS_SPI_NOT_READY:
        return "ABS SPI not ready";
      case INSUFFICIENT_RESOURCES:
        return "Insufficient resources";
      case INFEASIBLE_IO_NUM:
        return "Infeasible IO num";
      default:
        return "Unknown";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveEncoderError, Value>(value_, "Encoder", UNSTABLE_GAIN, INFEASIBLE_IO_NUM);
  }

  bool operator==(uint32_t v) const
  {
    return value_ == v;
  }
  bool operator==(ODriveEncoderError a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(ODriveEncoderError a) const
  {
    return value_ != a.value_;
  }

  uint32_t value_;
};

class ODriveControllerError
{
public:
  enum Value: uint32_t
  {
    NONE                       = 0x00000000,
    OVERSPEED                  = 0x00000001,
    INVALID_INPUT_MODE         = 0x00000002,
    UNSTABLE_GAIN              = 0x00000004,
    INVALID_MIRROR_AXIS        = 0x00000008,
    INVALID_LOAD_ENCODER       = 0x00000010,
    INVALID_ESTIMATE           = 0x00000020
  };

  ODriveControllerError() : value_(0)
  {
  }

  ODriveControllerError(uint32_t value) : value_(value)
  {
  }

  static std::string toString(Value value)
  {
    switch (value)
    {
      case NONE:
        return "None";
      case OVERSPEED:
        return "Overspeed";
      case INVALID_INPUT_MODE:
        return "Invalid input mode";
      case UNSTABLE_GAIN:
        return "Unstable gain";
      case INVALID_MIRROR_AXIS:
        return "Invalid mirror axis";
      case INVALID_LOAD_ENCODER:
        return "Invalid load encoder";
      case INVALID_ESTIMATE:
        return "Invalid estimate";
      default:
        return "Unknown";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveControllerError, Value>(value_, "Controller", OVERSPEED, INVALID_ESTIMATE);
  }

  bool operator==(uint32_t v) const
  {
    return value_ == v;
  }
  bool operator==(ODriveControllerError a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(ODriveControllerError a) const
  {
    return value_ != a.value_;
  }

  uint32_t value_;
};

class ODriveState : public MotorControllerState
{
public:
  ODriveState() = default;

  bool isOk() override
  {
    return axis_state_.value_ == ODriveAxisState::CLOSED_LOOP_CONTROL;
  }

  std::string getErrorStatus() override
  {
    std::ostringstream error_stream;
    std::string state = this->axis_state_.toString().c_str();

    error_stream << "State: " << state << ", " << axis_error_.toString() << ", " << motor_error_.toString()
                 << ", " << encoder_manager_error_.toString() << ", " << encoder_error_.toString() << ", "
                 << controller_error_.toString();
    return error_stream.str();
  }

  std::string getOperationalState() override
  {
    return axis_state_.toString();
  }

  ODriveAxisState axis_state_;
  ODriveAxisError axis_error_;
  ODriveMotorError motor_error_;
  ODriveEncoderManagerError encoder_manager_error_;
  ODriveEncoderError encoder_error_;
  ODriveControllerError controller_error_;

};
}  // namespace march

#endif  // MARCH_HARDWARE_ODRIVE_STATE_OF_OPERATION_H
