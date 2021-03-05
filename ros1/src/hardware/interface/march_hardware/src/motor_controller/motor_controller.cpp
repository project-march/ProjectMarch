#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/error/hardware_exception.h"
#include <memory>
namespace march
{
MotorController::MotorController(const Slave& slave, std::shared_ptr<AbsoluteEncoder> absolute_encoder,
                                 std::shared_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode)
  : Slave(slave), actuation_mode_(actuation_mode)
{
  if (incremental_encoder == nullptr && absolute_encoder_ == nullptr)
  {
    throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "A MotorController needs at least an incremental or an absolute encoder");
  }
  else
  {
    absolute_encoder_ = std::move(absolute_encoder);
    incremental_encoder_ = std::move(incremental_encoder);
  }
}

bool MotorController::isIncrementalEncoderMorePrecise() const
{
  if (!hasIncrementalEncoder())
  {
    return false;
  }
  if (!hasAbsoluteEncoder())
  {
    return true;
  }
  return incremental_encoder_->getRadiansPerBit() < absolute_encoder_->getRadiansPerBit();
}

double MotorController::getPosition()
{
  if (isIncrementalEncoderMorePrecise())
  {
    return getIncrementalPosition();
  }
  return getAbsolutePosition();
}

double MotorController::getPosition(bool absolute)
{
  if (absolute)
  {
    if (!hasAbsoluteEncoder())
    {
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get absolute position,"
                                                                        "the motor controller has no absolute encoder");
    }
    return getAbsolutePosition();
  }
  else
  {
    if (!hasIncrementalEncoder())
    {
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get incremental position, "
                                                                        "the motor controller has no incremental encoder");
    }
    return getIncrementalPosition();
  }
}

double MotorController::getVelocity()
{
  if (isIncrementalEncoderMorePrecise())
  {
    return getIncrementalVelocity();
  }
  return getAbsoluteVelocity();
}

double MotorController::getVelocity(bool absolute)
{
  if (absolute)
  {
    if (!hasAbsoluteEncoder())
    {
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get absolute velocity,"
                                                                        "the motor controller has no absolute encoder");
    }
    return getAbsoluteVelocity();
  }
  else
  {
    if (!hasIncrementalEncoder())
    {
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get incremental velocity,"
                                                                        "the motor controller has no incremental encoder");
    }
    return getIncrementalVelocity();
  }
}

ActuationMode MotorController::getActuationMode() const
{
  return actuation_mode_;
}

void MotorController::setActuationMode(ActuationMode actuation_mode)
{
  actuation_mode_ = actuation_mode;
};

bool MotorController::hasAbsoluteEncoder() const
{
  return absolute_encoder_ != nullptr;
}

bool MotorController::hasIncrementalEncoder() const
{
  return incremental_encoder_ != nullptr;
}

std::shared_ptr<AbsoluteEncoder> MotorController::getAbsoluteEncoder()
{
  return absolute_encoder_;
}

std::shared_ptr<IncrementalEncoder> MotorController::getIncrementalEncoder()
{
  return incremental_encoder_;
}

void MotorController::actuate(double target)
{
  if (actuation_mode_ == march::ActuationMode::position)
  {
    actuateRadians(target);
  }
  else if (actuation_mode_ == march::ActuationMode::torque)
  {
    actuateTorque(target);
  }
  else
  {
    throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE, "Actuation mode %s is not supported",
                                   actuation_mode_.toString().c_str());
  }
}


// Throw NotImplemented error by default for functions not part of the Minimum Viable Product

void MotorController::actuateRadians(double /*target_position*/)
{
  throw error::NotImplemented("actuateRadians", "current MotorController");
}

float MotorController::getMotorCurrent()
{
  throw error::NotImplemented("getMotorCurrent", "current MotorController");
}
float MotorController::getMotorControllerVoltage()
{
  throw error::NotImplemented("getMotorControllerVoltage", "current MotorController");
}
float MotorController::getMotorVoltage()
{
  throw error::NotImplemented("getMotorVoltage", "current MotorController");
}

double MotorController::getIncrementalPosition()
{
  throw error::NotImplemented("getIncrementalPosition", "current MotorController");
}

double MotorController::getIncrementalVelocity()
{
  throw error::NotImplemented("getIncrementalVelocity", "current MotorController");
}

}