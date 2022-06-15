// Copyright 2019 Project March.
#include "march_hardware/joint.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motor_controller_error.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/motor_controller_state.h"

#include <ros/ros.h>

#include <bitset>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace march {
Joint::Joint(std::string name, int net_number,
    std::unique_ptr<MotorController> motor_controller)
    : name_(std::move(name))
    , net_number_(net_number)
    , motor_controller_(std::move(motor_controller))
{
    if (!motor_controller_) {
        throw error::HardwareException(
            error::ErrorType::MISSING_MOTOR_CONTROLLER,
            "A Joint must have a MotorController");
    }
}

Joint::Joint(std::string name, int net_number,
    std::unique_ptr<MotorController> motor_controller,
    std::unique_ptr<TemperatureGES> temperature_ges)
    : name_(std::move(name))
    , net_number_(net_number)
    , motor_controller_(std::move(motor_controller))
    , temperature_ges_(std::move(temperature_ges))
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

std::optional<ros::Duration> Joint::prepareActuation()
{
    ROS_INFO("[%s] Preparing for actuation", this->name_.c_str());
    auto wait_duration = motor_controller_->prepareActuation();
    return wait_duration;
}

void Joint::enableActuation()
{
    ROS_INFO("[%s] Enabling for actuation", this->name_.c_str());
    motor_controller_->enableActuation();
}

void Joint::actuate(float target)
{
    motor_controller_->actuate(target);
}

void Joint::readFirstEncoderValues(bool operational_check)
{
    ROS_INFO("[%s] Reading first values", this->name_.c_str());

    // Preconditions check
    if (operational_check) {
        auto motor_controller_state = motor_controller_->getState();
        if (!motor_controller_state->isOperational()) {
            ROS_FATAL("[%s]: %s", this->name_.c_str(),
                motor_controller_state->getErrorStatus().value().c_str());
            throw error::HardwareException(
                error::ErrorType::PREPARE_ACTUATION_ERROR);
        }
    }

    if (motor_controller_->hasIncrementalEncoder()) {
        initial_incremental_position_
            = motor_controller_->getIncrementalPosition();
    }
    if (motor_controller_->hasAbsoluteEncoder()) {
        initial_absolute_position_ = motor_controller_->getAbsolutePosition();
        uint32_t position_iu
            = motor_controller_->getAbsoluteEncoder()->positionRadiansToIU(
                initial_absolute_position_);

        if (operational_check) {
            if (!motor_controller_->getAbsoluteEncoder()->isWithinHardLimitsIU(
                    position_iu)) {
                throw error::HardwareException(
                    error::ErrorType::OUTSIDE_HARD_LIMITS,
                    "Joint %s is outside hard limits, value is %d, limits are "
                    "[%d, "
                    "%d]",
                    name_.c_str(), position_iu,
                    motor_controller_->getAbsoluteEncoder()
                        ->getLowerHardLimitIU(),
                    motor_controller_->getAbsoluteEncoder()
                        ->getUpperHardLimitIU());
            }
        }
        position_ = initial_absolute_position_;
    }
    ROS_INFO("[%s] Read first values", this->name_.c_str());
}

void Joint::readEncoders(const ros::Duration& elapsed_time)
{
    if (this->receivedDataUpdate()) {
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
            double new_incremental_position
                = motor_controller_->getIncrementalPosition();
            position_ = initial_absolute_position_
                + (new_incremental_position - initial_incremental_position_);
        } else {
            position_ = motor_controller_->getAbsolutePosition();
        }
        velocity_ = motor_controller_->getVelocity();
    } else {
        ROS_WARN("Data was not updated within %.3fs, using old data",
            elapsed_time.toSec());
    }
}

double Joint::getPosition() const
{
    return position_;
}

double Joint::getVelocity() const
{
    return velocity_;
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
        throw error::HardwareException(
            error::ErrorType::MISSING_TEMPERATURE_GES,
            "Cannot get TemperatureGES of a Joint that does not have a "
            "TemperatureGES");
    }
}

} // namespace march
