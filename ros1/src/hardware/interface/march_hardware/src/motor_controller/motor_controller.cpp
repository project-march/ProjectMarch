#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/slave.h"
#include <memory>
namespace march
{
MotorController::MotorController(const Slave& slave, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
                                 std::unique_ptr<IncrementalEncoder> incremental_encoder,
                                 ActuationMode actuation_mode)
  : Slave(slave)
  , absolute_encoder_(std::move(absolute_encoder))
  , incremental_encoder_(std::move(incremental_encoder))
  , actuation_mode_(actuation_mode)
{
}

bool MotorController::isIncrementalEncoderMorePrecise() const
{
  return incremental_encoder_->getRadPerBit() < absolute_encoder_->getRadPerBit();
}

ActuationMode MotorController::getActuationMode() const
{
  return actuation_mode_;
}
}