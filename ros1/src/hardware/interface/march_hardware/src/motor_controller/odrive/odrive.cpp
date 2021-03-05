// Copyright 2018 Project March.
#include "march_hardware/motor_controller/odrive/odrive.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"
#include <march_hardware/motor_controller/motor_controller_state.h>
#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motion_error.h"
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
  if (this->absolute_encoder_ == nullptr || this->incremental_encoder_ == nullptr)
  {
    throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
                                   "An ODrive needs either an incremental or an absolute encoder");
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

void ODrive::actuateRadians(double /*target_position*/)
{
  throw std::logic_error("actuateRadians is not implemented for ODrive");
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
  auto imc_state = std::make_shared<ODriveState>();
  return imc_state;
}

float ODrive::getTorque()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualTorque, axis_number_)).f;
}
float ODrive::getMotorCurrent()
{
  throw std::logic_error("getMotorCurrent is not implemented for ODrive");
}
float ODrive::getMotorControllerVoltage()
{
  throw std::logic_error("getMotorControllerVoltage is not implemented for ODrive");
}
float ODrive::getMotorVoltage()
{
  throw std::logic_error("getMotorVoltage is not implemented for ODrive");
}

bool ODrive::initSdo(SdoSlaveInterface& /*sdo*/, int /*cycle_time*/)
{
  // No action is needed as the DieBoSlave make sure actuation is ready when etherCAT connection is made
  return false;
}

void ODrive::reset(SdoSlaveInterface& /*sdo*/)
{
  throw std::logic_error("reset is not implemented for ODrive");
}

double ODrive::getAbsolutePosition()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualPosition, axis_number_)).f;
}

double ODrive::getIncrementalPosition()
{
  throw std::logic_error("getIncrementalPosition is not implemented for ODrive");
}

double ODrive::getAbsoluteVelocity()
{
  return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualVelocity, axis_number_)).f;
}

double ODrive::getIncrementalVelocity()
{
  throw std::logic_error("getIncrementalVelocity is not implemented for ODrive");
}

ODriveAxisError ODrive::getAxisError()
{
  return ODriveAxisError(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisError, axis_number_)).ui);
}

ODriveMotorError ODrive::getMotorError()
{
  return ODriveMotorError(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorError, axis_number_)).ui);
}

ODriveEncoderManagerError ODrive::getEncoderManagerError()
{
  return ODriveEncoderManagerError(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::EncoderManagerError, axis_number_)).ui);
}

ODriveEncoderError ODrive::getEncoderError()
{
  return ODriveEncoderError(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::EncoderError, axis_number_)).ui);
}

ODriveControllerError ODrive::getControllerError()
{
  return ODriveControllerError(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ControllerError, axis_number_)).ui);
}
}