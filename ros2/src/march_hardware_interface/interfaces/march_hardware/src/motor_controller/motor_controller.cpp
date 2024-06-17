#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include "march_hardware/torque_sensor/torque_sensor.h"
#include <algorithm>
#include <memory>

namespace march {
MotorController::MotorController(const Slave& slave, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    std::unique_ptr<IncrementalEncoder> incremental_encoder, std::unique_ptr<TorqueSensor> torque_sensor,
    ActuationMode actuation_mode, bool use_low_level_for_position ,std::shared_ptr<march_logger::BaseLogger> logger)
    : Slave(slave)
    , absolute_encoder_(std::move(absolute_encoder))
    , incremental_encoder_(std::move(incremental_encoder))
    , torque_sensor_(std::move(torque_sensor))
    , actuation_mode_(actuation_mode)
    , use_low_level_for_position_(use_low_level_for_position)
    , logger_(std::move(logger))
{
    if (!incremental_encoder_) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "A MotorController needs an incremental encoder.");
    }
    if (!absolute_encoder_) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "A MotorController needs an absolute encoder.");
    }
}


std::chrono::nanoseconds MotorController::reset()
{
    return std::chrono::nanoseconds(0);
}

bool MotorController::useLowLevelForPosition() const
{
    return use_low_level_for_position_;
}

float MotorController::getPosition()
{
    if (useLowLevelForPosition()) {
        return getLowLevelPosition();
    }
    return getAbsolutePosition();
}


float MotorController::getVelocity()
{
    if (useLowLevelForPosition()) {
        return getIncrementalVelocity();
    }
    return getAbsoluteVelocity();
}

float MotorController::getAbsolutePosition()
{
    if (!hasAbsoluteEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get absolute position, the motor controller has no absolute encoder");
    }
    return getAbsolutePositionUnchecked();
}

float MotorController::getAbsoluteVelocity()
{
    if (!hasAbsoluteEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get absolute velocity, the motor controller has no absolute encoder");
    }
    return getAbsoluteVelocityUnchecked();
}

float MotorController::getIncrementalPosition()
{
    if (!hasIncrementalEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get incremental position, the motor controller has no incremental encoder");
    }
    return getIncrementalPositionUnchecked();
}

float MotorController::getIncrementalVelocity()
{
    if (!hasIncrementalEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get incremental velocity, the motor controller has no incremental encoder");
    }
    return getIncrementalVelocityUnchecked();
}

float MotorController::getTorque()
{
    if (!hasTorqueSensor()) {
        throw error::HardwareException(
            error::ErrorType::MISSING_ENCODER, "Cannot get torque, the motor controller has no torque sensor");
    } else {
        return getTorqueUnchecked();
    }
}

float MotorController::getLowLevelPosition()
{
    return getPosAbsRad();
}

double MotorController::getMotorControllerSpecificEffort(double joint_effort_command) const
{
    joint_effort_command = convertEffortToIUEffort(joint_effort_command);
    // Clamp effort to (-MAX_EFFORT, MAX_EFFORT)
    auto effort_limit = getTorqueLimit();
    return std::clamp(joint_effort_command, -effort_limit, effort_limit) * getMotorDirection();
}

ActuationMode MotorController::getActuationMode() const
{
    return actuation_mode_;
}

void MotorController::setActuationMode(ActuationMode actuation_mode)
{
    actuation_mode_ = actuation_mode;
}

bool MotorController::hasAbsoluteEncoder() const
{
    return absolute_encoder_ != nullptr;
}

bool MotorController::hasIncrementalEncoder() const
{
    return incremental_encoder_ != nullptr;
}

bool MotorController::hasTorqueSensor() const
{
    return torque_sensor_ != nullptr;
}

std::unique_ptr<AbsoluteEncoder>& MotorController::getAbsoluteEncoder()
{
    if (!hasAbsoluteEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get absolute encoder");
    }
    return absolute_encoder_;
}

std::unique_ptr<IncrementalEncoder>& MotorController::getIncrementalEncoder()
{
    if (!hasIncrementalEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get incremental encoder");
    }
    return incremental_encoder_;
}

std::unique_ptr<TorqueSensor>& MotorController::getTorqueSensor()
{
    if (!hasTorqueSensor()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER, "Cannot get torque sensor");
    }
    return torque_sensor_;
}

double MotorController::convertEffortToIUEffort(double joint_effort_command) const
{
    return joint_effort_command * effortMultiplicationConstant();
}

Encoder::Direction MotorController::getMotorDirection() const
{
    // Use the incremental encoder to determine motor direction
    return this->incremental_encoder_->getDirection();
}

double MotorController::effortMultiplicationConstant() const
{
    return 1.0;
}
} // namespace march