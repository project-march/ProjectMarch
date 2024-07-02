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

/***
 * This functions checks if the fuzzy control values are equal to one, when added together.
 * When this is the case, the joints van be actuated, and the target torque and position can be send to the ethercat.
 * @param target_position
 * @param target_torque
 * @param position_weight
 * @param torque_weight
 */
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
    logger_->info(logger_->fstring("[%s] Reading first values", this->name_.c_str()));

    // Preconditions check
    if (operational_check) {
        auto motor_controller_state = motor_controller_->getState();
        
        if (motor_controller_state->hasError()) {     
            logger_->fatal(logger_->fstring("[%s]: %s", this->name_.c_str(), motor_controller_state->getErrorStatus().c_str()));
            
            throw error::HardwareException(error::ErrorType::PREPARE_ACTUATION_ERROR);
        }
    }

    // TODO: outdated check since all joints have an incremental encoder > remove in the cleanup.
    if (motor_controller_->hasIncrementalEncoder()) {
        initial_incremental_position_ = motor_controller_->getIncrementalPosition();
        logger_->warn(logger_->fstring("Initial incremental position: %f", initial_incremental_position_));
    }

    // TODO: outdated check since all joints have an absolute encoder > remove in the cleanup.
    if (motor_controller_->hasAbsoluteEncoder()) {
        initial_absolute_position_ = motor_controller_->getAbsolutePosition();
        logger_->warn(logger_->fstring("Initial absolute position: %f", initial_absolute_position_));

        // TODO: this check should be removed once the motor controllers are properly communicating this information.
        if (initial_absolute_position_ == 0) {
            throw error::HardwareException(error::ErrorType::ABSOLUTE_ENCODER_ZERO,
                "Joint %s has an initial absolute position of 0, which is not allowed", name_.c_str());
        }

        position_ = initial_absolute_position_;
        logger_->warn(logger_->fstring("Current pos (iu ) is: %d", motor_controller_->getAbsoluteEncoder()->positionRadiansToIU(position_)));

        if (operational_check && !isWithinHardLimits()) {
            logger_->warn(logger_->fstring("Lower limit is: %d", motor_controller_->getAbsoluteEncoder()->getLowerHardLimitIU()));
            logger_->warn(logger_->fstring("Upper limit is: %d", motor_controller_->getAbsoluteEncoder()->getUpperHardLimitIU()));
            logger_->warn(logger_->fstring("Current pos (rad) is: %f", position_));
            logger_->warn(logger_->fstring("Current pos (iu ) is: %d", motor_controller_->getAbsoluteEncoder()->positionRadiansToIU(position_)));


            throw error::HardwareException(error::ErrorType::OUTSIDE_HARD_LIMITS,
                "Joint %s is outside hard limits, value is %f, limits are [%d, %d]", name_.c_str(), position_,
                motor_controller_->getAbsoluteEncoder()->getLowerHardLimitIU(),
                motor_controller_->getAbsoluteEncoder()->getUpperHardLimitIU());
        }
    }

    // Soon to be outdated check since all joints will have a torque sensor
    if (motor_controller_->hasTorqueSensor()) {
        initial_torque_ = motor_controller_->getTorque();
        logger_->warn(logger_->fstring("Initial torque: %f", initial_torque_));
        torque_ = initial_torque_;
        
        // if (motor_controller_->getTorqueSensor()->exceedsMaxTorque(initial_torque_)) {
        //     throw error::HardwareException(error::ErrorType::MAX_TORQUE_EXCEEDED,
        //         "Joint %s has an initial torque of %f, while initial torque can at most be absolute ", name_.c_str(),
        //         initial_torque_, motor_controller_->getTorqueSensor()->getMaxTorque());
        // }
    }
    logger_->info(logger_->fstring("[%s] Read initial values", this->name_.c_str()));
}

void Joint::readEncoders()
{
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_between_last_update = current_time - last_read_time_;

    if (this->receivedDataUpdate()) {
        last_read_time_ = std::chrono::steady_clock::now();
        /* Calculate position by using the base absolute position and adding
         * the difference between the initial incremental position and the new
         * incremental position
         *
         * Calculation example:
         * Say the first time the encoders are read they have the following
         * values:
         *  - absolute:     0.5
         *  - incremental:  0.7
         *
         *  Then the joint has an initial absolute position of 0.5 and an
         * initial incremental position of 0.7 Since the joint uses the absolute
         * encoder at startup, the initial position of the joint is 0.5
         *
         *  Say the second time the encoders are read they have the following
         * values:
         *  - absolute:     0.25
         *  - incremental:  0.4
         *
         *  If the incremental encoder of the joint is more precise, then we
         * should use that value.
         * The difference in incremental position is 0.4 - 0.7 = -0.3.
         * Hence the new joint position is 0.5 - 0.3 = 0.2.
         *
         *  If the absolute encoder of the joint is more precise, then we should
         * use that value This would give us a new joint position of 0.25
         */


        // TODO: check if really redundant
        if (motor_controller_-> useIncrementalEncoderForPosition()) {
            double new_incremental_position = motor_controller_->getIncrementalPosition();
            position_ = initial_absolute_position_ + (new_incremental_position - initial_incremental_position_);
        } else {
            position_ = motor_controller_->getAbsolutePosition();
        }

        velocity_ = motor_controller_->getVelocity();
        if (motor_controller_->hasTorqueSensor()) {
            torque_ = motor_controller_->getTorque();
            if (motor_controller_->getTorqueSensor()->exceedsMaxTorque(torque_)) {
                // throw error::HardwareException(error::ErrorType::MAX_TORQUE_EXCEEDED,
                //     "Joint %s has a torque of %f, while absolute max torque is %f", name_.c_str(), torque_,
                //     motor_controller_->getTorqueSensor()->getMaxTorque());
            }
        } else {
            logger_->info(logger_->fstring("No reading the torque sensors"));
        }
    } 
    // else {
    //     const std::chrono::milliseconds update_threshold{ 6 };
    //     if (time_between_last_update >= update_threshold) { // 0.01 = 10 milliseconds (one ethercat cycle is 8 ms).
    //         logger_->warn(
    //             logger_->fstring("Data was not updated within %d milliseconds (threshold: %d ms) for joint %s, using old data.",
    //                 this->name_.c_str(), time_between_last_update.count()));

    //     }
    // }
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

std::unique_ptr<MotorController>& Joint::getMotorController()
{
    return motor_controller_;
}

bool Joint::isWithinSoftLimits() const
{
    return motor_controller_->getAbsoluteEncoder()->isWithinSoftLimitsRadians(position_);
}

bool Joint::isWithinSoftErrorLimits() const
{
    return motor_controller_->getAbsoluteEncoder()->isWithinErrorSoftLimitsRadians(position_);
}

bool Joint::isWithinHardLimits() const
{
    return motor_controller_->getAbsoluteEncoder()->isWithinHardLimitsRadians(position_);
}

} // namespace march
