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
    int net_number, std::unique_ptr<MotorController> motor_controller,
    std::array<double, 3> position_pid,
    std::array<double, 3> torque_pid,
    std::shared_ptr<march_logger::BaseLogger> logger)
    :name_(std::move(name)),
    net_number_(net_number),
    last_read_time_(std::chrono::steady_clock::now()),
    motor_controller_(std::move(motor_controller)),
    position_pid(std::move(position_pid)),
    torque_pid(std::move(torque_pid)),
    logger_(std::move(logger))

{
    if (!motor_controller_) {
        throw error::HardwareException(
            error::ErrorType::MISSING_MOTOR_CONTROLLER, "Joint '%s' must have a MotorController.", name_.c_str());
    }
}

Joint::Joint(
    std::string name, int net_number,
    std::unique_ptr<MotorController> motor_controller,
    std::array<double, 3> position_pid,
    std::array<double, 3> torque_pid,
    std::unique_ptr<TemperatureGES> temperature_ges,
    std::shared_ptr<march_logger::BaseLogger> logger)
    : name_(std::move(name)),
    net_number_(net_number),
    last_read_time_(std::chrono::steady_clock::now()),
    motor_controller_(std::move(motor_controller)),
    position_pid(std::move(position_pid)),
    torque_pid(std::move(torque_pid)),
    temperature_ges_(std::move(temperature_ges)),
    logger_(std::move(logger))
{
}

bool Joint::initSdo(int cycle_time)
{
    bool reset = false;
    reset |= motor_controller_->Slave::initSdo(cycle_time);
    if (hasTemperatureGES()) {
        reset |= getTemperatureGES()->initSdo(cycle_time);
    }
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
            logger_->fatal(logger_->fstring(
                "[%s]: %s", this->name_.c_str(), motor_controller_state->getErrorStatus().c_str()));
            throw error::HardwareException(error::ErrorType::PREPARE_ACTUATION_ERROR);
        }
    }

    // Outdated check since all joints have an incremental encoder
    if (motor_controller_->hasIncrementalEncoder()) {
        initial_incremental_position_ = motor_controller_->getIncrementalPosition();
        logger_->warn(logger_->fstring("Initial incremental position: %f", initial_incremental_position_));
    }

    // Outdated check since all joints have an absolute encoder
    if (motor_controller_->hasAbsoluteEncoder()) {
        initial_absolute_position_ = motor_controller_->getAbsolutePosition();
        logger_->warn(logger_->fstring("Initial absolute position: %f", initial_absolute_position_));

        // Check that used to be in the exo system interface 
        if (initial_absolute_position_ == 0) {
            throw error::HardwareException(error::ErrorType::ABSOLUTE_ENCODER_ZERO,
                "Joint %s has an initial absolute position of 0, which is not allowed", name_.c_str());
        }

        position_ = initial_absolute_position_;

        if (operational_check && !isWithinHardLimits()) {
            throw error::HardwareException(error::ErrorType::OUTSIDE_HARD_LIMITS,
                "Joint %s is outside hard limits, value is %d, limits are [%d, %d]", name_.c_str(), position_,
                motor_controller_->getAbsoluteEncoder()->getLowerHardLimitIU(),
                motor_controller_->getAbsoluteEncoder()->getUpperHardLimitIU());
        }
    }

    // Soon to be outdated check since all joints will have a torque sensor
    if (motor_controller_->hasTorqueSensor()) {
        initial_torque_ = motor_controller_->getTorque();
        logger_->warn(logger_->fstring("Initial torque: %f", initial_torque_));
        torque_ = initial_torque_;
        
        // Comment back in after debugging
        if (motor_controller_->getTorqueSensor()->exceedsMaxTorque(initial_torque_)) {
            throw error::HardwareException(error::ErrorType::MAX_TORQUE_EXCEEDED,
                "Joint %s has an initial torque of %f, while initial torque can at most be absolute ", name_.c_str(),
                initial_torque_, motor_controller_->getTorqueSensor()->getMaxTorque());
        }
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
        if (motor_controller_->isIncrementalEncoderMorePrecise()) {
            double new_incremental_position = motor_controller_->getIncrementalPosition();
            position_ = initial_absolute_position_ + (new_incremental_position - initial_incremental_position_);
        } else {
            position_ = motor_controller_->getAbsolutePosition();
        }
        velocity_ = motor_controller_->getVelocity();
        if (motor_controller_->hasTorqueSensor()) {
            torque_ = motor_controller_->getTorque();
            if (motor_controller_->getTorqueSensor()->exceedsMaxTorque(torque_)) {
                throw error::HardwareException(error::ErrorType::MAX_TORQUE_EXCEEDED,
                    "Joint %s has a torque of %f, while absolute max torque is %f", name_.c_str(), torque_,
                    motor_controller_->getTorqueSensor()->getMaxTorque());
            }
        } else {
            logger_->info(logger_->fstring("No reading the torque sensors"));
        }
    } else {
        if (time_between_last_update
            >= std::chrono::milliseconds { 10 }) { // 0.01 = 10 milliseconds (one ethercat cycle is 8 ms).
            logger_->warn(
                logger_->fstring("Data was not updated within %.3f milliseconds for joint %s, using old data.",
                    this->name_.c_str(), time_between_last_update.count()));
        }
    }
}

void Joint::sendPID()
{
    this->motor_controller_->sendPID(std::move(this->position_pid), std::move(this->torque_pid));
}

void Joint::setPositionPIDValues(const std::array<double, 3>& new_position_pid)
{
    position_pid = std::array<double, 3>(new_position_pid);
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

int Joint::getNetNumber() const
{
    return net_number_;
}

std::string Joint::getName() const
{
    return name_;
}

bool Joint::hasTemperatureGES() const
{
    return temperature_ges_ != nullptr;
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

std::unique_ptr<TemperatureGES>& Joint::getTemperatureGES()
{
    if (hasTemperatureGES()) {
        return temperature_ges_;
    } else {
        logger_->error("Cannot get TemperatureGES of a Joint that does not have a TemperatureGES");
        throw error::HardwareException(error::ErrorType::MISSING_TEMPERATURE_GES,
            "Cannot get TemperatureGES of a Joint that does not have a TemperatureGES");
    }
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
