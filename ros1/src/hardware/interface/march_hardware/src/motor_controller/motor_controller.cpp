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
                                 std::shared_ptr<IncrementalEncoder> incremental_encoder,
                                 ActuationMode actuation_mode)
  : Slave(slave)
  , actuation_mode_(actuation_mode)
{
  if (incremental_encoder == nullptr && absolute_encoder_ == nullptr)
  {
    throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
                                   "A MotorController needs at least an incremental or an absolute encoder");
  }
  else
  {
    absolute_encoder_ = std::move(absolute_encoder);
    incremental_encoder_ = std::move(incremental_encoder);
  }
}

bool MotorController::initialize(int cycle_time)
{
  return this->initSdo(cycle_time);
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
  return incremental_encoder_->getRadPerBit() < absolute_encoder_->getRadPerBit();
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
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get absolute position");
    }
    return getAbsolutePosition();
  }
  else
  {
    if (!hasIncrementalEncoder())
    {
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get incremental position");
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
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get absolute velocity");
    }
    return getAbsoluteVelocity();
  }
  else
  {
    if (!hasIncrementalEncoder())
    {
      throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get incremental velocity");
    }
    return getIncrementalVelocity();
  }
}

ActuationMode MotorController::getActuationMode() const
{
  return actuation_mode_;
}

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

}