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
  MotorController(const Slave& slave, std::shared_ptr<AbsoluteEncoder> absolute_encoder,
                  std::shared_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode);

  double getPosition();
  double getVelocity();
  double getPosition(bool absolute);
  double getVelocity(bool absolute);

  ActuationMode getActuationMode() const;

  /*
   * Transform the ActuationMode to a number that is understood by the MotorController
   * @return the mode number that belong to the ActuationMode
   */
  virtual unsigned int getActuationModeNumber() const = 0;

  virtual double getTorque() = 0;
  virtual float getMotorCurrent() = 0;
  virtual float getMotorControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;

  virtual void actuateRadians(double target_position) = 0;
  virtual void actuateTorque(double target_effort) = 0;

  virtual void prepareActuation() = 0;
  bool initialize(int cycle_time);

  /**
   * Get whether the incremental encoder is more precise than the absolute encoder
   * @return true if the incremental encoder has a higher resolution than the absolute encoder, false otherwise
   */
  bool isIncrementalEncoderMorePrecise() const;

  bool hasAbsoluteEncoder() const;
  std::shared_ptr<AbsoluteEncoder> getAbsoluteEncoder();

  bool hasIncrementalEncoder() const;
  std::shared_ptr<IncrementalEncoder> getIncrementalEncoder();

  /**
   * Get the most recent states of the motor controller, i.e. all data that is read from the controller at every
   * communication cycle.
   * @return A MotorControllerState object containing all data read from the motor controller at every communication
   * cycle.
   */
  virtual std::unique_ptr<MotorControllerState> getState() = 0;

  /** @brief Override comparison operator */
  friend bool operator==(const MotorController& lhs, const MotorController& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && *lhs.absolute_encoder_ == *rhs.absolute_encoder_ &&
           *lhs.incremental_encoder_ == *rhs.incremental_encoder_ &&
           lhs.actuation_mode_.getValue() == rhs.actuation_mode_.getValue();
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const MotorController& motor_controller)
  {
    return os << "slaveIndex: " << motor_controller.getSlaveIndex() << ", "
              << "incrementalEncoder: " << *motor_controller.incremental_encoder_ << ", "
              << "absoluteEncoder: " << *motor_controller.absolute_encoder_
              << "actuationMode" << motor_controller.actuation_mode_.toString();
  }

protected:
  virtual double getAbsolutePosition() = 0;
  virtual double getIncrementalPosition() = 0;

  virtual double getAbsoluteVelocity() = 0;
  virtual double getIncrementalVelocity() = 0;

  std::shared_ptr<AbsoluteEncoder> absolute_encoder_ = nullptr;
  std::shared_ptr<IncrementalEncoder> incremental_encoder_ = nullptr;
  ActuationMode actuation_mode_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_MOTOR_CONTROLLER_H
