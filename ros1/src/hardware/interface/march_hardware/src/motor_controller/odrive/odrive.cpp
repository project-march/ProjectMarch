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

#include <ros/ros.h>

namespace march {
ODrive::ODrive(const Slave& slave, ODriveAxis axis,
    std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    ActuationMode actuation_mode)
    : MotorController(slave, std::move(absolute_encoder), actuation_mode)
    , axis_(axis)
{
    if (!absolute_encoder_) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "An ODrive needs an absolute encoder");
    }
}

void ODrive::prepareActuation()
{
    // No action is needed as the DieBoSlave makes sure actuation is ready when
    // etherCAT connection is made
}

void ODrive::actuateTorque(float target_torque)
{
    bit32 write_torque = { .f = target_torque };
    this->write32(
        ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TargetTorque, axis_),
        write_torque);
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

std::unique_ptr<MotorControllerState> ODrive::getState()
{
    auto state = std::make_unique<ODriveState>();

    // Set general attributes
    state->absolute_position_iu_ = getAbsolutePositionIU();
    state->absolute_velocity_iu_ = getAbsoluteVelocityIU();

    state->absolute_position_ = getAbsolutePositionUnchecked();
    state->absolute_velocity_ = getAbsoluteVelocityUnchecked();

    // Set ODrive specific attributes
    state->axis_state_ = getAxisState();
    state->axis_error_ = getAxisError();
    state->motor_error_ = getMotorError();
    state->encoder_manager_error_ = getEncoderManagerError();
    state->encoder_error_ = getEncoderError();
    state->controller_error_ = getControllerError();
    return state;
}

float ODrive::getTorque()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::ActualTorque, axis_))
        .f;
}

bool ODrive::initSdo(SdoSlaveInterface& /*sdo*/, int /*cycle_time*/)
{
    // No action is needed as the DieBoSlave make sure actuation is ready when
    // etherCAT connection is made
    return false;
}

void ODrive::reset(SdoSlaveInterface& /*sdo*/)
{
    // TODO: implement
}

ODriveAxisState ODrive::getAxisState()
{
    // TODO: implement
    return ODriveAxisState::CLOSED_LOOP_CONTROL;
}

float ODrive::getAbsolutePositionIU()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::ActualPosition, axis_))
        .f;
}

float ODrive::getAbsoluteVelocityIU()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::ActualVelocity, axis_))
        .f;
}

float ODrive::getAbsolutePositionUnchecked()
{
    return (float)this->getAbsoluteEncoder()->toRadians(
        getAbsolutePositionIU(), /*use_zero_position=*/true);
}

float ODrive::getAbsoluteVelocityUnchecked()
{
    return (float)this->getAbsoluteEncoder()->toRadians(
        getAbsoluteVelocityIU(), /*use_zero_position=*/false);
}

uint32_t ODrive::getAxisError()
{
    return this
        ->read32(
            ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisError, axis_))
        .ui;
}

uint32_t ODrive::getMotorError()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::MotorError, axis_))
        .ui;
}

uint32_t ODrive::getEncoderManagerError()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::EncoderManagerError, axis_))
        .ui;
}

uint32_t ODrive::getEncoderError()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::EncoderError, axis_))
        .ui;
}

uint32_t ODrive::getControllerError()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::ControllerError, axis_))
        .ui;
}

// Throw NotImplemented error by default for functions not part of the Minimum
// Viable Product
void ODrive::actuateRadians(float /*target_position*/)
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

float ODrive::getIncrementalPositionUnchecked()
{
    throw error::NotImplemented("getIncrementalPosition", "ODrive");
}

float ODrive::getIncrementalVelocityUnchecked()
{
    throw error::NotImplemented("getIncrementalVelocity", "ODrive");
}

} // namespace march
