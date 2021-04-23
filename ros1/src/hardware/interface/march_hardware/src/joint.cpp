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
Joint::Joint(std::string name, int net_number, bool allow_actuation,
    std::unique_ptr<MotorController> motor_controller)
    : name_(std::move(name))
    , net_number_(net_number)
    , allow_actuation_(allow_actuation)
    , motor_controller_(std::move(motor_controller))
{
    if (!motor_controller_) {
        throw error::HardwareException(
            error::ErrorType::MISSING_MOTOR_CONTROLLER,
            "A Joint must have a MotorController");
    }
}

Joint::Joint(std::string name, int net_number, bool allow_actuation,
    std::unique_ptr<MotorController> motor_controller,
    std::unique_ptr<TemperatureGES> temperature_ges)
    : name_(std::move(name))
    , net_number_(net_number)
    , allow_actuation_(allow_actuation)
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

void Joint::prepareActuation()
{
    if (!canActuate()) {
        throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE,
            "Failed to prepare joint %s for actuation", this->name_.c_str());
    }
    ROS_INFO("[%s] Preparing for actuation", this->name_.c_str());
    motor_controller_->prepareActuation();
    ROS_INFO("[%s] Successfully prepared for actuation", this->name_.c_str());

    if (motor_controller_->hasIncrementalEncoder()) {
        previous_incremental_position_
            = motor_controller_->getIncrementalPosition();
    }
    if (motor_controller_->hasAbsoluteEncoder()) {
        position_ = motor_controller_->getAbsolutePosition();
    } else {
        position_ = previous_incremental_position_;
    }
    velocity_ = 0;
}

void Joint::actuate(double target)
{
    if (!this->canActuate()) {
        throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE,
            "Joint %s is not allowed to actuate", this->name_.c_str());
    }
    motor_controller_->actuate(target);
}

void Joint::readEncoders(const ros::Duration& elapsed_time)
{
    if (this->receivedDataUpdate()) {
        if (motor_controller_->isIncrementalEncoderMorePrecise()) {
            double new_incremental_position
                = motor_controller_->getIncrementalPosition();
            position_
                += (new_incremental_position - previous_incremental_position_);
            previous_incremental_position_ = new_incremental_position;
        } else {
            position_ = motor_controller_->getAbsolutePosition();
        }
        velocity_ = motor_controller_->getVelocity();
    } else {
        // Update positions with velocity from last time step
        position_ += velocity_ * elapsed_time.toSec();
        previous_incremental_position_ += velocity_ * elapsed_time.toSec();
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

void Joint::setAllowActuation(bool allow_actuation)
{
    this->allow_actuation_ = allow_actuation;
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

bool Joint::canActuate() const
{
    return allow_actuation_;
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
