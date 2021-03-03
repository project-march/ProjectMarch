// Copyright 2018 Project March.
#include "march_hardware/motor_controller/odrive/odrive.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"
#include <march_hardware/motor_controller/motor_controller_state.h>
#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motion_error.h"
#include "march_hardware/ethercat/pdo_types.h"

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
ODrive::ODrive(const Slave& slave, std::shared_ptr<AbsoluteEncoder> absolute_encoder,
               std::shared_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode)
  : MotorController(slave, std::move(absolute_encoder), std::move(incremental_encoder), actuation_mode){};

void ODrive::prepareActuation()
{
  //TODO: implement
}

void ODrive::actuateRadians(double /*target_position*/)
{
  //TODO: implement
}

void ODrive::actuateTorque(double /*target_torque*/)
{
  //TODO: implement
}

unsigned int ODrive::getActuationModeNumber() const
{
  //TODO: implement
  return 0;
}

std::shared_ptr<MotorControllerState> ODrive::getState()
{
  auto imc_state = std::make_shared<ODriveState>();
  return imc_state;
}

float ODrive::getTorque()
{
  //TODO: implement
  return -1;
}
float ODrive::getMotorCurrent()
{
  //TODO: implement
  return -1;
}
float ODrive::getMotorControllerVoltage()
{
  //TODO: implement
  return -1;
}
float ODrive::getMotorVoltage()
{
  return -1;
}

bool ODrive::initSdo(SdoSlaveInterface& /*sdo*/, int /*cycle_time*/)
{
  //TODO: implement
  return false;
}

void ODrive::reset(SdoSlaveInterface& /*sdo*/)
{
  //TODO: implement
}

double ODrive::getAbsolutePosition()
{
  //TODO: implement
  return -1;
}

double ODrive::getIncrementalPosition()
{
  //TODO: implement
  return -1;
}

double ODrive::getAbsoluteVelocity()
{
  //TODO: implement
  return -1;
}

double ODrive::getIncrementalVelocity()
{
  //TODO: implement
  return -1;
}
}