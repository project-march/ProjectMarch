// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_IMOTIONCUBE_STATE_H
#define MARCH_HARDWARE_IMOTIONCUBE_STATE_H

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
    NONE = 0,
    ERROR = 1,
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
      case ERROR:
        return "Error";
      default:
        return "Unknown axis error";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveAxisError, Value>(value_, "Axis", ERROR, ERROR);
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
  enum Value: uint32_t
  {
    NONE = 0,
    ERROR = 1,
  };

  ODriveMotorError() : value_(0)
  {
  }

  ODriveMotorError(uint32_t value) : value_(value)
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
        return "Unknown motor error";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveMotorError, Value>(value_, "Motor", ERROR, ERROR);
  }

  bool operator==(uint16_t v) const
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

  uint32_t value_;
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

  bool operator==(uint16_t v) const
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
    NONE = 0,
    ERROR = 1
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
      case ERROR:
        return "Error";
      default:
        return "Unknown encoder error";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveEncoderError, Value>(value_, "Encoder", ERROR, ERROR);
  }

  bool operator==(uint16_t v) const
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
    NONE = 0,
    ERROR = 1
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
      case Value::NONE:
        return "None";
      case Value::ERROR:
        return "Error";
      default:
        return "Unknown controller error";
    }
  }

  std::string toString()
  {
    return errorsToString<ODriveControllerError, Value>(value_, "Controller", ERROR, ERROR);
  }

  bool operator==(uint16_t v) const
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
    return axis_state_.value_ == march::ODriveAxisState::CLOSED_LOOP_CONTROL;
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

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATE_OF_OPERATION_H
