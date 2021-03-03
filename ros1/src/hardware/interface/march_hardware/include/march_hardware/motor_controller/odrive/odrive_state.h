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
    switch (value_)
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
};
}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATE_OF_OPERATION_H
