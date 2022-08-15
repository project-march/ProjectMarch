// Copyright 2018 Project March.
#include "march_hardware/motor_controller/odrive/odrive.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motor_controller_error.h"
#include "march_hardware/ethercat/odrive_pdo_map.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"
#include <march_hardware/motor_controller/motor_controller_state.h>

#include "march_hardware/motor_controller/actuation_mode.h"

#include <bitset>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>

//#define DEBUG_EFFORT

namespace march {
ODrive::ODrive(const Slave& slave, ODriveAxis axis, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    std::unique_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode, bool index_found,
    unsigned int motor_kv, bool is_incremental_encoder_more_precise, std::shared_ptr<march_logger::BaseLogger> logger)
    : MotorController(slave, std::move(absolute_encoder), std::move(incremental_encoder), actuation_mode,
        is_incremental_encoder_more_precise, std::move(logger))
    , axis_(axis)
    , index_found_(index_found)
    , torque_constant_(KV_TO_TORQUE_CONSTANT / (float)motor_kv)
{
}

std::chrono::nanoseconds ODrive::reset()
{
    setAxisState(ODriveAxisState::CLEAR_ALL_ERRORS);
    return std::chrono::seconds { 1 };
}

std::chrono::nanoseconds ODrive::prepareActuation()
{
    if (!index_found_ && getAxisState() != ODriveAxisState::CLOSED_LOOP_CONTROL) {
        setAxisState(ODriveAxisState::ENCODER_INDEX_SEARCH);
        return std::chrono::seconds { 10 };
    } else {
        return std::chrono::nanoseconds(0);
    }
}

void ODrive::enableActuation()
{
    if (getAxisState() != ODriveAxisState::CLOSED_LOOP_CONTROL) {
        setAxisState(ODriveAxisState::CLOSED_LOOP_CONTROL);
    }

    // Reset target torque
    actuateTorque(/*target_effort=*/0.0);
}

void ODrive::waitForState(ODriveAxisState target_state)
{
    auto current_state = getAxisState();
    while (current_state != target_state) {
        logger_->info(logger_->fstring(
            "Waiting for '%s', currently in '%s'", target_state.toString().c_str(), current_state.toString().c_str()));

        usleep(/*__useconds=*/1000);
        current_state = getAxisState();
    }
}

void ODrive::actuateTorque(float target_effort)
{
    if (target_effort > EFFORT_LIMIT || target_effort < -EFFORT_LIMIT) {
        throw error::HardwareException(error::ErrorType::TARGET_TORQUE_EXCEEDS_MAX_TORQUE,
            "Target effort of %f exceeds effort limit of %f", target_effort, EFFORT_LIMIT);
    }

    float target_torque = target_effort * torque_constant_;
    logger_->debug(logger_->fstring("Effort: %f", target_effort));
    logger_->debug(logger_->fstring("Torque: %f", target_torque));
    bit32 write_torque {};
    write_torque.f = target_torque;
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TargetTorque, axis_), write_torque);
}

int ODrive::getActuationModeNumber() const
{
    /* These values are used to set the control mode of the ODrive.
     * They were retrieved from
     * https://github.com/odriverobotics/ODrive/blob/devel/Firmware/odrive-interface.yaml
     * Look for 'ODrive.Controller.ControlMode'.
     * The index in the list then represents the number corresponding to that
     * actuation mode. If the actuation mode is unknown this will return -1.
     */
    switch (actuation_mode_.getValue()) {
        case ActuationMode::torque:
            return 1;
        case ActuationMode::position:
            return 3;
        default:
            return -1;
    }
}

bool ODrive::requiresUniqueSlaves() const
{
    return false;
}

std::unique_ptr<MotorControllerState> ODrive::getState()
{
    auto state = std::make_unique<ODriveState>();

    // Set general attributes
    state->motor_current_ = getMotorCurrent();
    state->temperature_ = getTemperature();

    if (hasAbsoluteEncoder()) {
        state->absolute_position_iu_ = getAbsolutePositionIU();
        state->absolute_position_ = getAbsolutePositionUnchecked();
        state->absolute_velocity_ = getAbsoluteVelocityUnchecked();
    }
    if (hasIncrementalEncoder()) {
        state->incremental_position_iu_ = getIncrementalPositionIU();
        state->incremental_velocity_iu_ = getIncrementalVelocityIU();
        state->incremental_position_ = getIncrementalPositionUnchecked();
        state->incremental_velocity_ = getIncrementalVelocityUnchecked();
    }

    // Set ODrive specific attributes
    state->axis_state_ = getAxisState();
    state->axis_error_ = getAxisError();
    state->motor_error_ = getMotorError();
    state->dieboslave_error_ = getDieBOSlaveError();
    state->encoder_error_ = getEncoderError();
    state->controller_error_ = getControllerError();

    return state;
}

float ODrive::getTorque()
{
    return getMotorCurrent() * torque_constant_;
}

float ODrive::getTemperature()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Temperature, axis_)).f;
}

ODriveAxisState ODrive::getAxisState()
{
    return ODriveAxisState(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisState, axis_)).ui);
}

int32_t ODrive::getAbsolutePositionIU()
{
    int32_t iu_value = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualPosition, axis_)).i;
    if (iu_value == 0) {
        logger_->fatal("Absolute encoder value is 0 (Check the encoder cable, or flash the odrive).");
    }
    switch (absolute_encoder_->getDirection()) {
        case Encoder::Direction::Positive:
            return iu_value;
        case Encoder::Direction::Negative:
            return this->absolute_encoder_->getTotalPositions() - iu_value;
        default:
            throw error::HardwareException(error::ErrorType::INVALID_ENCODER_DIRECTION);
    }
}

int32_t ODrive::getIncrementalPositionIU()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorPosition, axis_)).i
        * incremental_encoder_->getDirection();
}

float ODrive::getIncrementalVelocityIU()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualVelocity, axis_)).f
        * (float)incremental_encoder_->getDirection();
}

float ODrive::getAbsolutePositionUnchecked()
{
    return (float)this->getAbsoluteEncoder()->positionIUToRadians(getAbsolutePositionIU());
}

float ODrive::getAbsoluteVelocityUnchecked()
{
    // The current ODrive firmware doesn't support absolute encoder velocity
    return (float)getIncrementalVelocityUnchecked();
}

float ODrive::getIncrementalPositionUnchecked()
{
    return (float)this->getIncrementalEncoder()->positionIUToRadians(getIncrementalPositionIU());
}

float ODrive::getIncrementalVelocityUnchecked()
{
    return (float)this->getIncrementalEncoder()->velocityIUToRadians(getIncrementalVelocityIU());
}

float ODrive::getMotorCurrent()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ActualCurrent, axis_)).f
        * (float)getMotorDirection();
}

float ODrive::getActualEffort()
{
    return getMotorCurrent();
}

uint32_t ODrive::getAxisError()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisError, axis_)).ui;
}

uint32_t ODrive::getMotorError()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorError, axis_)).ui;
}

uint32_t ODrive::getDieBOSlaveError()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::DieBOSlaveError, axis_)).ui;
}

uint32_t ODrive::getEncoderError()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::EncoderError, axis_)).ui;
}

uint32_t ODrive::getControllerError()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ControllerError, axis_)).ui;
}

void ODrive::setAxisState(ODriveAxisState state)
{
    bit32 write_struct {};
    write_struct.ui = state.value_;
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::RequestedState, axis_), write_struct);
}

double ODrive::getEffortLimit() const
{
    return EFFORT_LIMIT;
}

// Throw NotImplemented error by default for functions not part of the Minimum
// Viable Product

void ODrive::actuateRadians(float /*target_position*/)
{
    throw error::NotImplemented("actuateRadians", "ODrive");
}

float ODrive::getMotorControllerVoltage()
{
    throw error::NotImplemented("getMotorControllerVoltage", "ODrive");
}

float ODrive::getMotorVoltage()
{
    throw error::NotImplemented("getMotorVoltage", "ODrive");
}

} // namespace march
