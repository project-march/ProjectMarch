// Copyright 2019 Project March.
#include "march_hardware/joint.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motor_controller_error.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/motor_controller_state.h"

#include <bitset>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#define MAX_UPDATE_INTERVAL 9
namespace march {
Joint::Joint(
    std::string name,
    std::unique_ptr<MotorController> motor_controller,
    std::array<double, 3> position_gains,
    std::array<double, 2> torque_gains,
    std::shared_ptr<march_logger::BaseLogger> logger)
    :name_(std::move(name)),
    last_read_time_(std::chrono::steady_clock::now()),
    motor_controller_(std::move(motor_controller)),
    position_gains(std::move(position_gains)),
    torque_gains(std::move(torque_gains)),
    logger_(std::move(logger))

{
    if (!motor_controller_) {
        throw error::HardwareException(
            error::ErrorType::MISSING_MOTOR_CONTROLLER, "Joint '%s' must have a MotorController.", name_.c_str());
    }
}

bool Joint::initSdo(int cycle_time)
{
    bool reset = false;
    reset |= motor_controller_->Slave::initSdo(cycle_time);
    return reset;
}

std::chrono::nanoseconds Joint::prepareActuation()
{
    logger_->info(logger_->fstring("[%s] Preparing for actuation", this->name_.c_str()));
    auto wait_duration = motor_controller_->prepareActuation();
    return wait_duration;
}

void Joint::enableActuation()
{
    logger_->info(logger_->fstring("[%s] Enabling for actuation", this->name_.c_str()));
    motor_controller_->enableActuation();
}

void Joint::actuate(float target_position, float target_torque, float position_weight, float torque_weight)
{
    float total_fuzzy = torque_weight + position_weight;
    if (std::fabs(total_fuzzy - 1.0F) > std::numeric_limits<float>::epsilon()) {
        throw error::HardwareException(error::ErrorType::TOTAL_FUZZY_NOT_ONE,
            "Total weight exceeds value of one. With fuzzy position: %f and fuzzy torque: %f: ", position_weight,
            torque_weight);
    }
    motor_controller_->actuateTorque(target_torque, torque_weight);
    motor_controller_->actuateRadians(target_position, position_weight);
}

void Joint::readFirstEncoderValues(bool operational_check)
{
    logger_->info(logger_->fstring("[%s] Reading first encoder values", this->name_.c_str()));

    if (operational_check && hasMotorControllerError()) {
        throw error::HardwareException(error::ErrorType::PREPARE_ACTUATION_ERROR);
    }
    
    initial_incremental_position_ = motor_controller_->getIncrementalPosition();
    logger_->warn(logger_->fstring("Initial incremental position: %f", initial_incremental_position_));

    initial_absolute_position_ = motor_controller_->getAbsolutePosition();
    logger_->warn(logger_->fstring("Initial absolute position: %f", initial_absolute_position_));
    
    position_ = motor_controller_-> useLowLevelForPosition() ? motor_controller_->getLowLevelPosition() : motor_controller_->getAbsolutePosition();

    if (operational_check && !isWithinHardLimits()) {
    
        throw error::HardwareException(error::ErrorType::OUTSIDE_HARD_LIMITS,
            "Joint %s is outside hard limits"
            "\n\tPosition in radians: %g."
            "\n\tHard limits: [%g, %g] radians."
            "\n\tPosition in IU: %d."
            "\n\tHard limits: [%d, %d] IU.",
            this->name_.c_str(), position_,
            motor_controller_->getAbsoluteEncoder()->positionIUToRadians(motor_controller_->getAbsoluteEncoder()->getLowerHardLimitIU()),
            motor_controller_->getAbsoluteEncoder()->positionIUToRadians(motor_controller_->getAbsoluteEncoder()->getUpperHardLimitIU()),
            motor_controller_->getAbsoluteEncoder()->positionRadiansToIU(position_),
            motor_controller_->getAbsoluteEncoder()->getLowerHardLimitIU(),
            motor_controller_->getAbsoluteEncoder()->getUpperHardLimitIU());
    }
    
    initial_torque_ = motor_controller_->getTorque();
    logger_->warn(logger_->fstring("Initial torque: %f", initial_torque_));
    torque_ = initial_torque_;
        
    if (motor_controller_->getTorqueSensor()->exceedsMaxTorque(initial_torque_)) {
        throw error::HardwareException(error::ErrorType::MAX_TORQUE_EXCEEDED,
            "Joint %s has an initial torque of %f, while initial torque can at most be absolute ", name_.c_str(),
            initial_torque_, motor_controller_->getTorqueSensor()->getMaxTorque());
    }     

    logger_->info(logger_->fstring("[%s] Read initial encoder values, and joint is within its hard limits.", this->name_.c_str()));
}

bool Joint::hasMotorControllerError() {
    auto motor_controller_state = motor_controller_->getState();
    if (motor_controller_state->hasError()) {     
        logger_->fatal(logger_->fstring("[%s]: %s", this->name_.c_str(), motor_controller_state->getErrorStatus().c_str()));
        return true;
    }
    return false;
}

void Joint::readEncoders()
{
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_between_last_update = current_time - last_read_time_;

    if (!this->receivedDataUpdate()) {
        handleNoDataUpdate(time_between_last_update);
        return;
    }
    
    last_read_time_ = std::chrono::steady_clock::now();
    position_ = motor_controller_-> useLowLevelForPosition() ? motor_controller_->getLowLevelPosition() : motor_controller_->getAbsolutePosition();
    velocity_ = motor_controller_->getVelocity();

    if (motor_controller_->hasTorqueSensor()) {
        torque_ = motor_controller_->getTorque();
        if (motor_controller_->getTorqueSensor()->exceedsMaxTorque(torque_)) {
            throw error::HardwareException(error::ErrorType::MAX_TORQUE_EXCEEDED,
                "Joint %s has a torque of %f, while absolute max torque is %f", name_.c_str(), torque_,
                motor_controller_->getTorqueSensor()->getMaxTorque());
        } 
    } 
}

void Joint::sendPID()
{
    this->motor_controller_->sendPID(std::move(this->position_gains), std::move(this->torque_gains));
}

void Joint::setPositionPIDValues(const std::array<double, 3>& new_position_gains)
{
    position_gains = std::array<double, 3>(new_position_gains);
} 

double Joint::getPosition() const
{
    return position_;
}

double Joint::getVelocity() const
{
    return velocity_;
}

double Joint::getTorque() const
{
    return torque_;
}

std::string Joint::getName() const
{
    return name_;
}

bool Joint::receivedDataUpdate()
{
    auto new_state = motor_controller_->getState();

    bool data_updated;
    if (!previous_state_.has_value()) {
        data_updated = true;
    } else {
        data_updated = !(*(previous_state_.value()) == *new_state);
    }
    previous_state_ = std::move(new_state);
    return data_updated;
}

void Joint::handleNoDataUpdate(std::chrono::duration<double> time_between_last_update) {
    const std::chrono::milliseconds max_update_interval {MAX_UPDATE_INTERVAL}; 
    if (time_between_last_update >= max_update_interval) {
        logger_->warn(
            logger_->fstring("Data was not updated within %.3f seconds for joint %s, using old data.",
                this->name_.c_str(), time_between_last_update.count()));
    }
}

std::unique_ptr<MotorController>& Joint::getMotorController()
{
    return motor_controller_;
}

bool Joint::isWithinSoftLimits() const
{
    return motor_controller_->getAbsoluteEncoder()->isWithinSoftLimitsRadians(position_);
}

bool Joint::isWithinHardLimits() const
{
    return motor_controller_->getAbsoluteEncoder()->isWithinHardLimitsRadians(position_);
}

} // namespace march
