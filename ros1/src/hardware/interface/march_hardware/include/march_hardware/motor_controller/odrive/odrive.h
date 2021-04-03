// Copyright 2021 Project March.

#ifndef MARCH_HARDWARE_ODRIVE_H
#define MARCH_HARDWARE_ODRIVE_H

#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/ethercat/pdo_map.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"

#include <memory>
#include <unordered_map>
#include <string>

namespace march
{
class ODrive : public MotorController
{
public:
  /**
   * Constructs an IMotionCube with an incremental and absolute encoder.
   *
   * @param slave slave of the IMotionCube
   * @param absolute_encoder pointer to absolute encoder, required so cannot be nullptr
   * @param incremental_encoder pointer to incremental encoder, required so cannot be nullptr
   * @param actuation_mode actuation mode in which the IMotionCube must operate
   * @throws std::invalid_argument When an absolute or incremental encoder is nullptr.
   */
  ODrive(const Slave& slave, int axis_number, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
         std::unique_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode);
  ODrive(const Slave& slave, int axis_number, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
         ActuationMode actuation_mode);

  ~ODrive() noexcept override = default;

  // Override functions for actuating the ODrive
  void prepareActuation() override;
  void actuateTorque(double target_torque) override;
  void actuateRadians(double target_position) override;

  // Transform the ActuationMode to a number that is understood by the ODrive
  unsigned int getActuationModeNumber() const override;

  // Get a full description of the state of the ODrive
  std::unique_ptr<MotorControllerState> getState() override;

  // Getters for specific information about the state of the motor and the ODrive
  float getTorque() override;
  float getMotorCurrent() override;
  float getMotorControllerVoltage() override;
  float getMotorVoltage() override;

protected:
  // Override protected functions from Slave class
  bool initSdo(SdoSlaveInterface& sdo, int cycle_time) override;
  void reset(SdoSlaveInterface& sdo) override;

  // Override protected functions from MotorController class
  double getAbsolutePosition() override;
  double getIncrementalPosition() override;
  double getAbsoluteVelocity() override;
  double getIncrementalVelocity() override;

private:
  // Set the ODrive in a certain axis state
//  void goToAxisState(ODriveAxisState target_state);

//  TODO: Look at getAxisState()
//  ODriveAxisState getAxisState();

  float getAbsolutePositionIU();
  float getAbsoluteVelocityIU();

  uint32_t getAxisError();
  uint32_t getMotorError();
  uint32_t getEncoderManagerError();
  uint32_t getEncoderError();
  uint32_t getControllerError();


  int axis_number_;
};

}  // namespace march

#endif  // MARCH_HARDWARE_ODRIVE_H
