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

#include <rclcpp/rclcpp.hpp>
#include <bitset>
#include <memory>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>

// Allows easy debugging of all incoming errors
// #define DEBUG_MODE


namespace march {
ODrive::ODrive(const Slave& slave, ODriveAxis axis, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    std::unique_ptr<IncrementalEncoder> incremental_encoder, std::unique_ptr<TorqueSensor> torque_sensor,
    ActuationMode actuation_mode, bool use_low_level_for_position, bool index_found, std::shared_ptr<march_logger::BaseLogger> logger)
    : MotorController(slave, std::move(absolute_encoder), std::move(incremental_encoder), std::move(torque_sensor),
        actuation_mode, std::move(use_low_level_for_position) , std::move(logger))
    , axis_(axis)
    , index_found_(index_found)
{
}

// TODO: reintegrate this state on the mdrives
std::chrono::nanoseconds ODrive::reset()
{
    // setAxisState(ODriveAxisState::CLEAR_ALL_ERRORS);
    return std::chrono::seconds { 1 };
}

std::chrono::nanoseconds ODrive::prepareActuation()
{
    if (!index_found_ && getAxisState() != ODriveAxisState::CLOSED_LOOP_CONTROL) {

        setAxisState(ODriveAxisState::ENCODER_INDEX_SEARCH);
        logger_->info("Initializing the encoder index search.");    

        return std::chrono::seconds {20};

    } else {
        return std::chrono::nanoseconds(0);
    }
}

// Before enabling actuation, a joint should have gone through its entire callibration sequence, after which it goes back to idle.
void ODrive::enableActuation()
{
    auto current_state = getAxisState();
    auto target_state = ODriveAxisState::CLOSED_LOOP_CONTROL;
    if (getAxisState() != ODriveAxisState::CLOSED_LOOP_CONTROL && getAxisState() == ODriveAxisState::IDLE) {
        setAxisState(ODriveAxisState::CLOSED_LOOP_CONTROL);
        RCLCPP_WARN(rclcpp::get_logger("enableActuation"),"MDrive currently in state: %s, and setting state to %s",current_state.toString().c_str(),ODriveAxisState::toString(target_state).c_str());
    } else {
        RCLCPP_WARN(rclcpp::get_logger("enableActuation"),"MDrive currently in state: %s, should be in state 8.",current_state.toString().c_str());
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

void ODrive::sendTargetPosition(float target_position)
{
    bit32 write_position {};
    write_position.f = target_position;
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TargetPosition, axis_), write_position);
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
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TargetPosition, axis_), write_position);
    bit32 write_fuzzy {};
    write_fuzzy.f = fuzzy_weight;
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::FuzzyPosition, axis_), write_fuzzy);
}

void ODrive::sendPID(std::array<double, 3> pos_pid, std::array<double, 2> tor_pid)
{
    bit32 write_position_p {};
    bit32 write_position_i {};
    bit32 write_position_d {};

    write_position_p.f = static_cast<float>(pos_pid[0]);
    write_position_i.f = static_cast<float>(pos_pid[1]);
    write_position_d.f = static_cast<float>(pos_pid[2]);

    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::PositionP, axis_), write_position_p);
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::PositionI, axis_), write_position_i);
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::PositionD, axis_), write_position_d);

    bit32 write_torque_p {};
    bit32 write_torque_d {};

    write_torque_p.f = static_cast<float>(tor_pid[0]);
    write_torque_d.f = static_cast<float>(tor_pid[1]);

    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TorqueP, axis_), write_torque_p);
    this->write32(ODrivePDOmap::getMOSIByteOffset(ODriveObjectName::TorqueD, axis_), write_torque_d);
}


// Not used currently
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

    state->pos_abs_rad_ = getPosAbsRad();
    state->AIE_absolute_position_ = getAIEAbsolutePositionRad();
    state->check_sum_ = getCheckSum();

    // Set ODrive specific attributes
    state->axis_state_ = getAxisState();
    state->odrive_error_ = getODriveError();
    state->axis_error_ = getAxisError();
    state->motor_error_ = getMotorError();
    state->encoder_error_ = getEncoderError();
    state->torquesensor_error_ = getTorqueSensorError();
    state->controller_error_ = getControllerError();

    return state;
}


float ODrive::getMotorTemperature()
{
    float motor_temp = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorTemperature, axis_)).f;
    float motor_temp_check = 100;
    if (motor_temp > motor_temp_check) {
        RCLCPP_ERROR(rclcpp::get_logger("getMotorTemperature"), "Motor temperature is %f, the MotorTemperature object's bits are probably scrambled like some tasty eggs.", motor_temp);
    }
    return motor_temp;
}

// TODO: get rid of this object.
float ODrive::getOdriveTemperature()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::OdriveTemperature, axis_)).f;
}

// TODO: create check for faulty states
ODriveAxisState ODrive::getAxisState()
{
    return ODriveAxisState(this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisState, axis_)).ui);
}

uint32_t ODrive::getAbsolutePositionIU()
{
    uint32_t abs_pos_iu = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AbsolutePosition, axis_)).ui;
    if (abs_pos_iu == 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(50)); // Sleep for 50ms to give the MDrive time to update the encoder value
        abs_pos_iu = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AbsolutePosition, axis_)).ui;
        if (abs_pos_iu == 0){
            logger_->fatal("Absolute encoder value is 0 (Check the encoder cable, or flash the odrive).");
        }
    }

    uint32_t abs_pos_check = 1048576; // Better would be to differentiate between the different joints but this is a good start
    if (abs_pos_iu > abs_pos_check) {
        RCLCPP_ERROR(rclcpp::get_logger("getAbsolutePositionIU"), "Absolute encoder value is %u, the AbsolutePosition object's bits are probably scrambled like some tasty eggs.", abs_pos_iu);
    }

    switch (absolute_encoder_->getDirection()) {
        case Encoder::Direction::Positive:
            return abs_pos_iu;
        case Encoder::Direction::Negative:
            return this->absolute_encoder_->getTotalPositions() - abs_pos_iu;
        default:
            throw error::HardwareException(error::ErrorType::INVALID_ENCODER_DIRECTION);
    }
}

int32_t ODrive::getIncrementalPositionIU()
{
    int32_t inc_pos_iu = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ShadowCount, axis_)).i;
    int32_t inc_pos_check = 1e6;
    if (abs(inc_pos_iu) > inc_pos_check) {
        RCLCPP_ERROR(rclcpp::get_logger("getIncrementalPositionIU"), "Incremental encoder value is %d, the ShadowCount object's bits are probably scrambled like some tasty eggs.", inc_pos_iu);
    }
    return inc_pos_iu*incremental_encoder_->getDirection();
}

float ODrive::getIncrementalVelocityIU()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorVelocity, axis_)).f
        * (float)incremental_encoder_->getDirection();
}

float ODrive::getAIEAbsolutePositionRad()
{
    float aie_pos = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AIEAbsolutePosition, ODriveAxis::None)).f;
    float aie_pos_check = 0.4;
    if (abs(aie_pos) > aie_pos_check) {
        RCLCPP_ERROR_ONCE(rclcpp::get_logger("getAIEAbsolutePositionRad"), "AIEAbsolutePosition value is %f, the AIEAbsolutePosition object's bits are probably scrambled like some tasty eggs.", aie_pos);
    }
    return aie_pos; 
}

uint32_t ODrive::getCheckSum()
{
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::CheckSumMISO, ODriveAxis::None)).ui;
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
    return this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Torque, axis_)).f;
}

float ODrive::getPosAbsRad()
{
    float pos_abs_rad = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::PosAbsRad, axis_)).f;
    float pos_abs_rad_check = 2;
    if (abs(pos_abs_rad) > pos_abs_rad_check) {
        RCLCPP_ERROR(rclcpp::get_logger("getPosAbsRad"), "PosAbsRad value is %f, the PosAbsRad object's bits are probably scrambled like some tasty eggs.", pos_abs_rad);
    }
    return pos_abs_rad;
}

float ODrive::getMotorCurrent()
{
    float motor_current = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::Current, axis_)).f;
    float motor_current_check = 50;
    if (motor_current > motor_current_check) {
        RCLCPP_ERROR(rclcpp::get_logger("getMotorCurrent"), "Motor current is %f, the Current object's bits are probably scrambled like some tasty eggs.", motor_current);
    }
    return motor_current * (float)getMotorDirection();
}

uint32_t ODrive::getODriveError()
{
    uint32_t error = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ODriveError, axis_)).ui;
#ifdef DEBUG_MODE
    if (error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODrive"), "ODrive error: %u", error);
    }
#endif
    return error;
}

uint32_t ODrive::getAxisError()
{
    uint32_t error = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::AxisError, axis_)).ui;
#ifdef DEBUG_MODE
    if (error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODrive"), "Axis error: %u", error);
    }
#endif
    return error;
}

uint64_t ODrive::getMotorError()
{
    uint64_t error = this->read64(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::MotorError, axis_)).ui;
#ifdef DEBUG_MODE
    if (error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODrive"), "Motor error: %llu", error);
    }
#endif
    return error;
}

uint32_t ODrive::getEncoderError()
{
    uint32_t error = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::EncoderError, axis_)).ui;
#ifdef DEBUG_MODE
    if (error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODrive"), "Encoder error: %u", error);
    }
#endif
    return error;
}

uint32_t ODrive::getTorqueSensorError()
{
    uint32_t error = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::TorqueSensorError, axis_)).ui;
#ifdef DEBUG_MODE
    if (error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODrive"), "Torque sensor error: %u", error);
    }
#endif
    return error;
}

uint32_t ODrive::getControllerError()
{
    uint32_t error = this->read32(ODrivePDOmap::getMISOByteOffset(ODriveObjectName::ControllerError, axis_)).ui;
#ifdef DEBUG_MODE
    if (error != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ODrive"), "Controller error: %u", error);
    }
#endif
    return error;
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
