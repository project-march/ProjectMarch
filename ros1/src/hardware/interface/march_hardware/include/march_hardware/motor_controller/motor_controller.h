// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_H
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/slave.h"
#include <string>
#include <memory>

namespace march
{
class MotorController : public Slave
{
public:
  MotorController(const Slave& slave, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
                  std::unique_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode);

  double getAngle();
  double getVelocity();
  virtual double getTorque() = 0;

  ActuationMode getActuationMode() const;

  /*
   * Transform the ActuationMode to a number that is understood by the MotorController
   * @return the mode number that belong to the ActuationMode
   */
  virtual unsigned int getActuationModeNumber() const = 0;

  virtual float getMotorCurrent() = 0;
//  virtual float getMotorControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;

  virtual void actuateRad(double target_rad) = 0;
  virtual void actuateTorque(double target_torque_ampere) = 0;

  virtual void prepareActuation() = 0;
  virtual bool initialize(int cycle_time) = 0;
  virtual void reset() = 0;

  /**
 * Get whether the incremental encoder is more precise than the absolute encoder
 * @return true if the incremental encoder has a higher resolution than the absolute encoder, false otherwise
 */
  bool isIncrementalEncoderMorePrecise() const;

  /**
   * Get the most recent states of the motor controller, i.e. all data that is read from the controller at every
   * communication cycle.
   * @return A MotorControllerState object containing all data read from the motor controller at every communication
   * cycle.
   */
  std::unique_ptr<MotorControllerState> getState();
private:
  std::unique_ptr<AbsoluteEncoder> absolute_encoder_;
  std::unique_ptr<IncrementalEncoder> incremental_encoder_;
  ActuationMode actuation_mode_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_MOTOR_CONTROLLER_H
