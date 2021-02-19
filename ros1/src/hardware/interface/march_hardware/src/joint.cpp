// Copyright 2019 Project March.
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/joint.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motion_error.h"
#include <march_hardware/motor_controller/motor_controller_state.h>

#include <ros/ros.h>

#include <bitset>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace march
{
Joint::Joint(std::string name, int net_number, bool allow_actuation, std::shared_ptr<IMotionCube> motor_controller)
  : name_(std::move(name))
  , net_number_(net_number)
  , allow_actuation_(allow_actuation)
  , motor_controller_(std::move(motor_controller))
{
}

Joint::Joint(std::string name, int net_number, bool allow_actuation, std::shared_ptr<IMotionCube> motor_controller,
             std::shared_ptr<TemperatureGES> temperature_ges)
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
  reset |= this->motor_controller_->Slave::initSdo(cycle_time);
  if (this->hasTemperatureGES())
  {
    reset |= this->temperature_ges_->initSdo(cycle_time);
  }
  return reset;
}

void Joint::prepareActuation()
{
  if (!this->canActuate())
  {
    throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE, "Failed to prepare joint %s for actuation",
                                   this->name_.c_str());
  }
  ROS_INFO("[%s] Preparing for actuation", this->name_.c_str());
  motor_controller_->prepareActuation();
  ROS_INFO("[%s] Successfully prepared for actuation", this->name_.c_str());

  this->previous_incremental_position_ = motor_controller_->getPosition(false);
  this->position_ = motor_controller_->getPosition(true);
  this->velocity_ = 0;
}

void Joint::actuate(double target)
{
  if (!this->canActuate())
  {
    throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE, "Joint %s is not allowed to actuate",
                                   this->name_.c_str());
  }
  motor_controller_->actuate(target);
}

void Joint::readEncoders(const ros::Duration& elapsed_time)
{
  if (this->receivedDataUpdate())
  {
    if (motor_controller_->isIncrementalEncoderMorePrecise())
    {
      double new_incremental_position = motor_controller_->getPosition();
      position_ += (new_incremental_position - previous_incremental_position_);
      previous_incremental_position_ = new_incremental_position;
    }
    else
    {
      position_ = motor_controller_->getPosition();
    }
    velocity_ = motor_controller_->getVelocity();
  }
  else
  {
    // Update positions with velocity from last time step
    position_ += velocity_ * elapsed_time.toSec();
    previous_incremental_position_ += velocity_ * elapsed_time.toSec();
  }
}

double Joint::getPosition() const
{
  return this->position_;
}

double Joint::getVelocity() const
{
  return this->velocity_;
}

void Joint::setAllowActuation(bool allow_actuation)
{
  this->allow_actuation_ = allow_actuation;
}

int Joint::getNetNumber() const
{
  return this->net_number_;
}

std::string Joint::getName() const
{
  return this->name_;
}

bool Joint::hasTemperatureGES() const
{
  return this->temperature_ges_ != nullptr;
}

bool Joint::canActuate() const
{
  return this->allow_actuation_;
}

bool Joint::receivedDataUpdate()
{
  std::unique_ptr<MotorControllerState> new_state = motor_controller_->getState();

  bool data_updated;
  if (previous_state_ == nullptr)
  {
    data_updated = true;
  }
  else
  {
    data_updated =  !(*previous_state_ == *new_state);
  }
  previous_state_ = std::move(new_state);
  return data_updated;
}

std::shared_ptr<MotorController> Joint::getMotorController()
{
  return motor_controller_;
}

std::shared_ptr<TemperatureGES> Joint::getTemperatureGES()
{
  return temperature_ges_;
}

}  // namespace march
