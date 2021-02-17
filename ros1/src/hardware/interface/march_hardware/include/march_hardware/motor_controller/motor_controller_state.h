// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H

#include <string>

namespace march
{
class MotorControllerState
{
public:
  MotorControllerState() = default;

  float motor_current_;
  float motor_voltage_;
  float motor_controller_voltage_;
  double absolute_angle_iu_;
  double incremental_angle_iu_;
  double absolute_velocity_iu_;
  double incremental_velocity_iu_;
  double absolute_angle_rad_;
  double incremental_angle_rad_;
  double absolute_velocity_rad_;
  double incremental_velocity_rad_;

  friend bool operator==(const MotorControllerState& lhs, const MotorControllerState& rhs)
  {
    return lhs.motor_current_ == rhs.motor_current_ && lhs.motor_voltage_ == rhs.motor_voltage_ &&
    lhs.absolute_angle_iu_ == rhs.absolute_angle_iu_ && lhs.incremental_angle_iu_ == rhs.incremental_angle_iu_ &&
    lhs.absolute_velocity_iu_ == rhs.absolute_velocity_iu_ && lhs.incremental_velocity_iu_ == rhs.incremental_velocity_iu_ &&
    lhs.absolute_angle_rad_ == rhs.absolute_angle_rad_ && lhs.incremental_angle_rad_ == rhs.incremental_angle_rad_ &&
    lhs.absolute_velocity_rad_ == rhs.absolute_velocity_rad_ && lhs.incremental_velocity_rad_ == rhs.incremental_velocity_rad_;
  }

  /**
   * Check whether the motor controller is in an error state
   * @return false if the motor controller is in error state, otherwise true
   */
  virtual bool isOk() = 0;
  /**
   * Get a string description of the state and error states of the motor controller
   * @return string describing the current state as well as the error state(s) of the motor controller
   */
  virtual std::string getErrorStatus() = 0;

  /**
   * Get a string description of the operational state
   * @return string describing the operational state
   */
  virtual std::string getOperationalState() = 0;
};

}  // namespace march

#endif  // MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
