// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H

#include <string>

namespace march
{
class MotorControllerStateOfOperation
{
public:
  MotorControllerStateOfOperation() = default;
  virtual std::string toString();
};


class MotorControllerState
{
public:
  MotorControllerState() = default;
  MotorControllerState(float motor_current, float motor_voltage,
                       double absolute_angle_iu, double incremental_angle_iu,
                       double absolute_velocity_iu, double incremental_velocity_iu,
                       double absolute_angle_rad, double incremental_angle_rad,
                       double absolute_velocity_rad, double incremental_velocity_rad,
                       MotorControllerStateOfOperation state_of_operation)
    : motor_current_(motor_current)
    , motor_voltage_(motor_voltage)
    , absolute_angle_iu_(absolute_angle_iu)
    , incremental_angle_iu_(incremental_angle_iu)
    , absolute_velocity_iu_(absolute_velocity_iu)
    , incremental_velocity_iu_(incremental_velocity_iu)
    , absolute_angle_rad_(absolute_angle_rad)
    , incremental_angle_rad_(incremental_angle_rad)
    , absolute_velocity_rad_(absolute_velocity_rad)
    , incremental_velocity_rad_(incremental_velocity_rad)
    , state_of_operation_(state_of_operation)
  {}

  float motor_current_;
  float motor_voltage_;
  double absolute_angle_iu_;
  double incremental_angle_iu_;
  double absolute_velocity_iu_;
  double incremental_velocity_iu_;
  double absolute_angle_rad_;
  double incremental_angle_rad_;
  double absolute_velocity_rad_;
  double incremental_velocity_rad_;
  MotorControllerStateOfOperation state_of_operation_;

  /**
   * Check whether the motor controller is in an error state
   * @return false if the motor controller is in error state, otherwise true
   */
  virtual bool checkState() = 0;
  /**
   * Get a string description of the state and error states of the motor controller
   * @return string describing the current state as well as the error state(s) of the motor controller
   */
  virtual std::string getErrorStatus() = 0;
};

}  // namespace march

#endif  // MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
