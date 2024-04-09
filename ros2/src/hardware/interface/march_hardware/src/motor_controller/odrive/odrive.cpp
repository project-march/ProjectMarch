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
    std::unique_ptr<IncrementalEncoder> incremental_encoder, std::unique_ptr<TorqueSensor> torque_sensor,
    ActuationMode actuation_mode, bool index_found, unsigned int motor_kv, bool is_incremental_encoder_more_precise,
    std::shared_ptr<march_logger::BaseLogger> logger)
    : MotorController(slave, std::move(absolute_encoder), std::move(incremental_encoder), std::move(torque_sensor),
        actuation_mode, is_incremental_encoder_more_precise, std::move(logger))
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
        return std::chrono::seconds { 20 };
    } else {
        return std::chrono::nanoseconds(0);
    }
}

// TODO: add check afterwards if ODrives are indeed in state 8
void ODrive::enableActuation()
{
    if (getAxisState() != ODriveAxisState::CLOSED_LOOP_CONTROL) {
        setAxisState(ODriveAxisState::CLOSED_LOOP_CONTROL);
    } else {
        logger_->info("ODrive state already in closed loop control");
    }

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

/*** This method writes the desired torque to the ethercat, together with the corresponding fuzzy control weight.
 *
 * First there is a check to see if the torque limit is being surpassed.
 * When this is not the case, the torque can be writen to the etherat.
 * @param target_torque
 * @param fuzzy_weight
 */
void ODrive::actuateTorque(float target_torque, float fuzzy_weight)
{
    if (target_torque > TORQUE_LIMIT || target_torque < -TORQUE_LIMIT) {
        throw error::HardwareException(error::ErrorType::TARGET_TORQUE_EXCEEDS_MAX_TORQUE,
            "Target torque of %f exceeds effort limit of %f", target_torque, TORQUE_LIMIT);
    }
    bit32 write_fuzzy {};
    write_fuzzy.f = fuzzy_weight;
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::FuzzyTorque, axis_), write_fuzzy);

    bit32 write_torque {};
    write_torque.f = target_torque;
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TargetTorque, axis_), write_torque);
}

/*** This method writes the desired position to the ethercat, together with the corresponding fuzzy control weight.
 *
 * First there is a check to see if the position is within the joint limits.
 * When this is not the case, the position can be writen to the etherat.
 * @param target_position
 * @param fuzzy_weight
 */
void ODrive::actuateRadians(float target_position, float fuzzy_weight)
{
    if (this->hasAbsoluteEncoder()
        && !this->absolute_encoder_->isValidTargetIU(
            this->getAbsolutePositionIU(), this->absolute_encoder_->positionRadiansToIU(target_position))) {
        throw error::HardwareException(error::ErrorType::INVALID_ACTUATE_POSITION,
            "Error in Odrive %i \nThe requested position is outside the limits, for requested position %f: ",
            this->getSlaveIndex(), target_position);
    }

    bit32 write_position {};
    write_position.f = target_position;
    // logger_->info(logger_->fstring(
    // "Sending position %f to the exo.", target_position));
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TargetPosition, axis_), write_position);
    bit32 write_fuzzy {};
    write_fuzzy.f = fuzzy_weight;
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::FuzzyPosition, axis_), write_fuzzy);
}

void ODrive::sendPID(std::array<double, 3> pos_pid, std::array<double, 3> tor_pid)
{
    auto offset
        = ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::PositionP, axis_); // TODO: fix this with ODrivePDOMap.
    for (double& i : pos_pid) {
        bit32 write_value {};
        // logger_->info(logger_->fstring("Sending PID value %f, with offset %d, to the exo.", i, offset));
        write_value.f = static_cast<float>(i);
        this->write32(offset, write_value);
        offset += 4;
    }

    offset = ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TorqueP, axis_); // TODO:fix this with ODrivePDOMap.
    for (double& i : tor_pid) {
        bit32 write_value {};
        write_value.f = static_cast<float>(i);
        this->write32(offset, write_value);
        offset += 4;
    }
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

// TODO: rename to something more descriptive.
std::unique_ptr<MotorControllerState> ODrive::getState()
{
    auto state = std::make_unique<ODriveState>();

    // Set general attributes
    state->motor_current_ = getMotorCurrent();
    state->motor_temperature_ = getMotorTemperature();
    state->motor_controller_temperature_ = getOdriveTemperature();

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
    state->odrive_error_ = getOdriveError();
    state->axis_error_ = getAxisError();
    state->motor_error_ = getMotorError();
    state->dieboslave_error_ = getDieBOSlaveError();
    state->encoder_error_ = getEncoderError();
    state->controller_error_ = getControllerError();

    return state;
}

// float ODrive::getTorque()
//{
//    return getMotorCurrent() * torque_constant_;
//}

float ODrive::getMotorTemperature()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorTemperature, axis_)).f;
}

float ODrive::getOdriveTemperature()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::OdriveTemperature, axis_)).f;
}

ODriveAxisState ODrive::getAxisState()
{
    // logger_->info("Reading axis state");
    return ODriveAxisState(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisState, axis_)).ui);
}

int32_t ODrive::getAbsolutePositionIU()
{
    int32_t iu_value = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AbsolutePosition, axis_)).i;
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
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ShadowCount, axis_)).i
        * incremental_encoder_->getDirection();
}

float ODrive::getIncrementalVelocityIU()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorVelocity, axis_)).f
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

float ODrive::getTorqueUnchecked()
{
    float measured_torque = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Torque, axis_)).f;
    return measured_torque;
}

float ODrive::getMotorCurrent()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Current, axis_)).f
        * (float)getMotorDirection();
}

float ODrive::getActualEffort()
{
    return getMotorCurrent();
}

uint32_t ODrive::getOdriveError()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::OdriveError, ODriveAxis::None)).ui;
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

double ODrive::getTorqueLimit() const
{
    return TORQUE_LIMIT;
}

// Throw NotImplemented error by default for functions not part of the Minimum
// Viable Product
float ODrive::getMotorControllerVoltage()
{
    throw error::NotImplemented("getMotorControllerVoltage", "ODrive");
}

float ODrive::getMotorVoltage()
{
    throw error::NotImplemented("getMotorVoltage", "ODrive");
}

} // namespace march
