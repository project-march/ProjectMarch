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
    torque_constant_ = KV_TO_TORQUE_CONSTANT / (float)motor_kv;
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
    : ODrive(slave, axis, std::move(absolute_encoder), actuation_mode,
        /*pre_calibrated=*/true, motor_kv)
{
}

void ODrive::prepareActuation()
{
    // if (!pre_calibrated_) {
    //     // Calibrate the ODrive first
    //     setAxisState(ODriveAxisState::FULL_CALIBRATION_SEQUENCE);
    //     waitForState(ODriveAxisState::IDLE);
    // }

    if (getAxisState() != ODriveAxisState::CLOSED_LOOP_CONTROL) {
        // Set the ODrive to closed loop control
        setAxisState(ODriveAxisState::CLOSED_LOOP_CONTROL);
    }
}

void ODrive::waitForState(ODriveAxisState target_state)
{
    auto current_state = getAxisState();
    while (current_state != target_state) {
        ROS_INFO("Waiting for '%s', currently in '%s'",
            target_state.toString().c_str(), current_state.toString().c_str());

        ros::Duration(/*t=*/1).sleep();
        current_state = getAxisState();
    }
}

void ODrive::actuateTorque(float target_effort)
{
    ROS_INFO("Effort: %f", target_effort);
    float target_torque
        = target_effort * torque_constant_ * (float)getMotorDirection();
    // ROS_INFO("Torque: %f", target_torque);
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

bool ODrive::hasUniqueSlaves() const
{
    return false;
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
    state->incremental_velocity_ = getIncrementalVelocityUnchecked();

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
    // Almost no action is needed as the DieBoSlave make sure actuation is ready
    // when etherCAT connection is made. Only need to wait a second for the
    // EtherCAT network to become available.
    //    ros::Duration(/*t=*/1).sleep();
    return false;
}

void ODrive::reset(SdoSlaveInterface& /*sdo*/)
{
}

ODriveAxisState ODrive::getAxisState()
{
    return ODriveAxisState(this->read32(ODrivePDOmap::getMISOByteOffset(
                                            ODriveObjectName::AxisState, axis_))
                               .ui);
}

int32_t ODrive::getAbsolutePositionIU()
{
    int32_t iu_value
        = this->read32(ODrivePDOmap::getMISOByteOffset(
                           ODriveObjectName::ActualPosition, axis_))
              .i;

    switch (absolute_encoder_->getDirection()) {
        case Encoder::Direction::Positive:
            return iu_value;
        case Encoder::Direction::Negative:
            return this->absolute_encoder_->getTotalPositions() - iu_value;
        default:
            throw error::HardwareException(
                error::ErrorType::INVALID_ENCODER_DIRECTION);
    }
}

int32_t ODrive::getIncrementalPositionIU()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(
                            ODriveObjectName::MotorPosition, axis_))
               .i
        * incremental_encoder_->getDirection();
}

float ODrive::getIncrementalVelocityIU()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(
                            ODriveObjectName::ActualVelocity, axis_))
               .f
        * (float)incremental_encoder_->getDirection();
}

float ODrive::getAbsolutePositionUnchecked()
{
    return (float)this->getAbsoluteEncoder()->positionIUToRadians(
        getAbsolutePositionIU());
}

float ODrive::getAbsoluteVelocityUnchecked()
{
    // The current ODrive firmware doesn't support absolute encoder velocity
    return (float)getIncrementalVelocityUnchecked();
}

float ODrive::getIncrementalPositionUnchecked()
{
    return (float)this->getIncrementalEncoder()->positionIUToRadians(
        getIncrementalPositionIU());
}

float ODrive::getIncrementalVelocityUnchecked()
{
    return (float)this->getIncrementalEncoder()->velocityIUToRadians(
        getIncrementalVelocityIU());
}

float ODrive::getMotorCurrent()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(
                            ODriveObjectName::ActualCurrent, axis_))
               .f
        * (float)getMotorDirection();
}

float ODrive::getActualEffort()
{
    return getMotorCurrent();
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

bool ODrive::isIncrementalEncoderMorePrecise() const
{
    return true;
}

Encoder::Direction ODrive::getMotorDirection() const
{
    // Use the incremental encoder to determine motor direction
    return this->incremental_encoder_->getDirection();
}

void ODrive::setAxisState(ODriveAxisState state)
{
    bit32 write_struct = { .ui = state.value_ };
    this->write32(ODrivePDOmap::getMOSIByteOffset(
                      ODriveObjectName::RequestedState, axis_),
        write_struct);
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
