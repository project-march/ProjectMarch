// Copyright 2018 Project March.
#include "march_hardware/motor_controller/odrive/odrive.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"
#include <march_hardware/motor_controller/motor_controller_state.h>
#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motor_controller_error.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/ethercat/pdo_map.h"

#include "march_hardware/motor_controller/actuation_mode.h"

#include <bitset>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>

#include <ros/ros.h>

namespace march
{
ODrive::ODrive(const Slave& slave, int axis_number, std::shared_ptr<AbsoluteEncoder> absolute_encoder,
               std::shared_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode)
  : MotorController(slave, std::move(absolute_encoder), std::move(incremental_encoder), actuation_mode)
  , axis_number_(axis_number)
{
  if (this->absolute_encoder_ == nullptr)
  {
    throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
                                   "An ODrive needs an absolute encoder");
  }
};

ODrive::ODrive(const Slave& slave, int axis_number, std::shared_ptr<AbsoluteEncoder> absolute_encoder,
               ActuationMode actuation_mode)
    : ODrive(slave, axis_number, std::move(absolute_encoder), nullptr, actuation_mode)
{};

void ODrive::prepareActuation()
{
  // No action is needed as the DieBoSlave make sure actuation is ready when etherCAT connection is made
}

void ODrive::actuateTorque(double target_torque)
{
  bit32 write_torque = {.f = (float) target_torque};
  this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TargetTorque, axis_number_), write_torque);
}

unsigned int ODrive::getActuationModeNumber() const
{
  switch(this->actuation_mode_.getValue())
  {
    case ActuationMode::position:
      return 3;
    case ActuationMode::torque:
      return 1;
    default:
      return 1;
  }
}

std::shared_ptr<MotorControllerState> ODrive::getState()
{
  auto state = std::make_shared<ODriveState>();

  //Set general attributes
  state->absolute_position_iu_ = getAbsolutePositionIU();
  state->absolute_velocity_iu_ = getAbsoluteVelocityIU();

  state->absolute_position_ = getAbsolutePosition();
  state->absolute_velocity_ = getAbsoluteVelocity();

  // Set ODrive specific attributes
//  state->axis_state_ = getAxisState(); // TODO: implement
  state->axis_error_ = getAxisError();
  state->motor_error_ = getMotorError();
  state->encoder_manager_error_ = getEncoderManagerError();
  state->encoder_error_ = getEncoderError();
  state->controller_error_ = getControllerError();
  return state;
}

float ODrive::getTorque()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualTorque, axis_number_)).f;
}

bool ODrive::initSdo(SdoSlaveInterface& /*sdo*/, int /*cycle_time*/)
{
  // No action is needed as the DieBoSlave make sure actuation is ready when etherCAT connection is made
  return false;
}

void ODrive::reset(SdoSlaveInterface& /*sdo*/)
{
  //TODO: implement
}

float ODrive::getAbsolutePositionIU()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualPosition, axis_number_)).f;
}

float ODrive::getAbsoluteVelocityIU()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualVelocity, axis_number_)).f;
}

double ODrive::getAbsolutePosition()
{
  return absolute_encoder_->toRadians(getAbsolutePositionIU(), true);
}

double ODrive::getAbsoluteVelocity()
{
  return absolute_encoder_->toRadians(getAbsoluteVelocityIU(), false);
}

uint32_t ODrive::getAxisError()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisError, axis_number_)).ui;
}

uint32_t ODrive::getMotorError()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorError, axis_number_)).ui;
}

uint32_t ODrive::getEncoderManagerError()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::EncoderManagerError, axis_number_)).ui;
}

uint32_t ODrive::getEncoderError()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::EncoderError, axis_number_)).ui;
}

uint32_t ODrive::getControllerError()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ControllerError, axis_number_)).ui;
}

// Throw NotImplemented error by default for functions not part of the Minimum Viable Product
void ODrive::actuateRadians(double /*target_position*/)
{
  throw error::NotImplemented("actuateRadians", "ODrive");
}

float ODrive::getMotorCurrent()
{
  throw error::NotImplemented("getMotorCurrent", "ODrive");
}

float ODrive::getMotorControllerVoltage()
{
  throw error::NotImplemented("getMotorControllerVoltage", "ODrive");
}

float ODrive::getMotorVoltage()
{
  throw error::NotImplemented("getMotorVoltage", "ODrive");
}

double ODrive::getIncrementalPosition()
{
  throw error::NotImplemented("getIncrementalPosition", "ODrive");
}

double ODrive::getIncrementalVelocity()
{
  throw error::NotImplemented("getIncrementalVelocity", "ODrive");
}
}