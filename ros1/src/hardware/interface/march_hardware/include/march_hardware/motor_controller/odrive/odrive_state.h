// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_IMOTIONCUBE_STATE_H
#define MARCH_HARDWARE_IMOTIONCUBE_STATE_H

#include <string>
#include "march_hardware/motor_controller/motor_controller_state.h"

namespace march
{
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

  static constexpr Value AllAxisErrors[]{
      ERROR
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
    std::stringstream ss;
    ss << "Axis errors: ";
    if (value_ == 0)
    {
      ss << "None";
    }
    else
    {
      ss << "[";
      for (unsigned int i = 0; i < sizeof(AllAxisErrors) / sizeof(uint32_t); ++i)
      {
        auto current_error = AllAxisErrors[i];
        if ((value_ & current_error) > 0)
        {
          ss << toString(current_error) << ", ";
        }
      }
      ss << "]";
    }
    return ss.str();
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

  static constexpr Value AllMotorErrors[]{
      ERROR
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
    std::stringstream ss;
    ss << "Motor errors: ";
    if (value_ == 0)
    {
      ss << "None";
    }
    else
    {
      ss << "[";
      for (unsigned int i = 0; i < sizeof(AllMotorErrors) / sizeof(uint32_t); ++i)
      {
        auto current_error = AllMotorErrors[i];
        if ((value_ & current_error) > 0)
        {
          ss << toString(current_error) << ", ";
        }
      }
      ss << "]";
    }
    return ss.str();
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

  static constexpr Value AllEncoderManagerErrors[]{
    ERROR
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
    std::stringstream ss;
    ss << "Encoder manager errors: ";
    if (value_ == 0)
    {
      ss << "None";
    }
    else
    {
      ss << "[";
      for (unsigned int i = 0; i < sizeof(AllEncoderManagerErrors) / sizeof(uint32_t); ++i)
      {
        auto current_error = AllEncoderManagerErrors[i];
        if ((value_ & current_error) > 0)
        {
          ss << toString(current_error) << ", ";
        }
      }
      ss << "]";
    }
    return ss.str();
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
    //TODO: implement
    return "";
  }

  std::string getOperationalState() override
  {
    return axis_state_.toString();
  }

  ODriveAxisState axis_state_;
  ODriveAxisError axis_error_;
  ODriveMotorError motor_error_;
  ODriveEncoderManagerError encoder_manager_error_;
  uint16_t encoder_error_;
  uint8_t controller_error_;

};
}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATE_OF_OPERATION_H
