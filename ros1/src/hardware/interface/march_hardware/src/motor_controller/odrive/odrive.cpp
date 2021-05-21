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
    std::unique_ptr<IncrementalEncoder> incremental_encoder,
    ActuationMode actuation_mode, bool pre_calibrated, unsigned int motor_kv)
    : MotorController(slave, std::move(absolute_encoder),
        std::move(incremental_encoder), actuation_mode)
    , axis_(axis)
    , pre_calibrated_(pre_calibrated)
{
    if (!absolute_encoder_) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "An ODrive needs an absolute encoder");
    }
    torque_constant_ = motor_kv * KV_TO_TORQUE_CONSTANT;
}

ODrive::ODrive(const Slave& slave, ODriveAxis axis,
    std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    ActuationMode actuation_mode, bool pre_calibrated, unsigned int motor_kv)
    : ODrive(slave, axis, std::move(absolute_encoder), nullptr, actuation_mode,
        pre_calibrated, motor_kv)
{
}

ODrive::ODrive(const Slave& slave, ODriveAxis axis,
    std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    ActuationMode actuation_mode, unsigned int motor_kv)
    : ODrive(slave, axis, std::move(absolute_encoder), actuation_mode, false, motor_kv)
{
}

void ODrive::prepareActuation()
{
    if (!pre_calibrated_) {
        // Calibrate the ODrive first
        setAxisState(ODriveAxisState::FULL_CALIBRATION_SEQUENCE);
        waitForState(ODriveAxisState::IDLE);
    }
    // Set the ODrive to closed loop control
    setAxisState(ODriveAxisState::CLOSED_LOOP_CONTROL);
    waitForState(ODriveAxisState::CLOSED_LOOP_CONTROL);

    auto odrive_state = getState();
    if (odrive_state->hasError()) {
        ROS_FATAL("%s", odrive_state->getErrorStatus().value().c_str());
        throw error::HardwareException(
            error::ErrorType::PREPARE_ACTUATION_ERROR);
    }
}

void ODrive::waitForState(ODriveAxisState target_state)
{
    auto current_state = getAxisState();
    while (current_state != target_state) {
        ROS_INFO("\tWaiting for '%s', currently in '%s'",
            target_state.toString().c_str(), current_state.toString().c_str());

        ros::Duration(1).sleep();
        current_state = getAxisState();
    }
}

void ODrive::actuateTorque(float target_effort)
{
    float target_torque = target_effort * torque_constant_;
    ROS_INFO_STREAM("Writing torque: " << target_torque);
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
    state->motor_current_ = getMotorCurrent();
    state->absolute_position_iu_ = getAbsolutePositionIU();
    state->incremental_position_iu_ = getIncrementalPositionIU();
    state->incremental_velocity_iu_ = getIncrementalVelocityIU();

    state->absolute_position_ = getAbsolutePositionUnchecked();
    state->absolute_velocity_ = getAbsoluteVelocityUnchecked();
    state->incremental_position_ = getIncrementalPositionUnchecked();
    state->incremental_position_ = getIncrementalPositionUnchecked();

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
    return getMotorCurrent() * torque_constant_;
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
    return ODriveAxisState(this->read32(ODrivePDOmap::getMISOByteOffset(
        ODriveObjectName::AxisState, axis_))
        .ui);
}

void ODrive::setAxisState(ODriveAxisState /* state */)
{
    return;
//    bit32 write_struct = { .ui = state.value_ };
//    this->write32(ODrivePDOmap::getMOSIByteOffset(
//                      ODriveObjectName::RequestedState, axis_),
//        write_struct);
}

int32_t ODrive::getAbsolutePositionIU()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::ActualPosition, axis_))
        .i;
}

float ODrive::getIncrementalPositionIU()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::MotorPosition, axis_))
        .f;
}

float ODrive::getIncrementalVelocityIU()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::ActualVelocity, axis_))
        .f;
}

float ODrive::getAbsolutePositionUnchecked()
{
    return this->getAbsoluteEncoder()->positionIUToRadians(
        getAbsolutePositionIU());
}

float ODrive::getAbsoluteVelocityUnchecked()
{
    // The current ODrive firmware doesn't support absolute encoder velocity
    return getIncrementalVelocityUnchecked();
}

float ODrive::getIncrementalPositionUnchecked()
{
    return this->getIncrementalEncoder()->positionIUToRadians(
        getIncrementalPositionIU());
}

float ODrive::getIncrementalVelocityUnchecked()
{
    return this->getIncrementalEncoder()->velocityIUToRadians(
        getIncrementalVelocityIU());
}

float ODrive::getMotorCurrent()
{
    return this
        ->read32(ODrivePDOmap::getMISOByteOffset(
            ODriveObjectName::ActualCurrent, axis_))
        .f;
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

float ODrive::getMotorControllerVoltage()
{
    throw error::NotImplemented("getMotorControllerVoltage", "ODrive");
}

float ODrive::getMotorVoltage()
{
    throw error::NotImplemented("getMotorVoltage", "ODrive");
}

} // namespace march
