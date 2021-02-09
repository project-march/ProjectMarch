// Copyright 2019 Project March.
#include "march_hardware/communication/ethercat/slave.h"
#include "march_hardware/joint.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motion_error.h"

#include <ros/ros.h>

#include <bitset>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace march
{
Joint::Joint(std::string name, int net_number) : name_(std::move(name)), net_number_(net_number)
{
}

Joint::Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<IMotionCube> imc)
  : name_(std::move(name)), net_number_(net_number), allow_actuation_(allow_actuation), imc_(std::move(imc))
{
}

Joint::Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<IMotionCube> imc,
             std::unique_ptr<TemperatureGES> temperature_ges)
  : name_(std::move(name))
  , net_number_(net_number)
  , allow_actuation_(allow_actuation)
  , imc_(std::move(imc))
  , temperature_ges_(std::move(temperature_ges))
{
}

bool Joint::initialize(int cycle_time)
{
  bool reset = false;
  if (this->hasIMotionCube())
  {
    reset |= this->imc_->Slave::initSdo(cycle_time);
  }
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
  this->imc_->goToOperationEnabled();
  ROS_INFO("[%s] Successfully prepared for actuation", this->name_.c_str());

  this->incremental_position_ = this->imc_->getAngleRadIncremental();
  this->absolute_position_ = this->imc_->getAngleRadAbsolute();
  this->position_ = this->absolute_position_;
  this->velocity_ = 0;
}

void Joint::resetIMotionCube()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
  }
  else
  {
    this->imc_->Slave::reset();
  }
}

void Joint::actuateRad(double target_position)
{
  if (!this->canActuate())
  {
    throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE, "Joint %s is not allowed to actuate",
                                   this->name_.c_str());
  }
  this->imc_->actuateRad(target_position);
}

void Joint::readEncoders(const ros::Duration& elapsed_time)
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return;
  }

  if (this->receivedDataUpdate())
  {
    const double incremental_position_change = this->imc_->getAngleRadIncremental() - this->incremental_position_;

    // Take the velocity and position from the encoder with the highest resolution.
    if (this->imc_->getIncrementalRadPerBit() < this->imc_->getAbsoluteRadPerBit())
    {
      this->velocity_ = this->imc_->getVelocityRadIncremental();
      this->position_ += incremental_position_change;
    }
    else
    {
      this->velocity_ = this->imc_->getVelocityRadAbsolute();
      this->position_ = this->imc_->getAngleRadAbsolute();
    }
    this->incremental_position_ += incremental_position_change;
    this->absolute_position_ = this->imc_->getAngleRadAbsolute();
  }
  else
  {
    // Update positions with velocity from last time step
    this->position_ += this->velocity_ * elapsed_time.toSec();
    this->incremental_position_ += this->velocity_ * elapsed_time.toSec();
    this->absolute_position_ += this->velocity_ * elapsed_time.toSec();
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

double Joint::getVoltageVelocity() const
{
  // For the underlying calculations, see:
  // https://en.wikipedia.org/wiki/Motor_constants#Motor_velocity_constant,_back_EMF_constant
  // https://www.motioncontroltips.com/faq-whats-relationship-voltage-dc-motor-output-speed/
  const double resistance = 0.05;
  const double velocity_constant = 355;
  const double rpm_to_rad = M_PI / 30;
  const double electric_constant = velocity_constant * rpm_to_rad;
  return (this->imc_->getMotorVoltage() + this->imc_->getMotorCurrent() * resistance) / electric_constant;
}

double Joint::getIncrementalPosition() const
{
  return this->incremental_position_;
}

double Joint::getAbsolutePosition() const
{
  return this->absolute_position_;
}

void Joint::actuateTorque(int16_t target_torque)
{
  if (!this->canActuate())
  {
    throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE, "Joint %s is not allowed to actuate",
                                   this->name_.c_str());
  }
  this->imc_->actuateTorque(target_torque);
}

int16_t Joint::getTorque()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getTorque();
}

int32_t Joint::getAngleIUAbsolute()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getAngleIUAbsolute();
}

int32_t Joint::getAngleIUIncremental()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getAngleIUIncremental();
}

double Joint::getVelocityIUAbsolute()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getVelocityIUAbsolute();
}

double Joint::getVelocityIUIncremental()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getVelocityIUIncremental();
}

float Joint::getTemperature()
{
  if (!this->hasTemperatureGES())
  {
    ROS_WARN("[%s] Has no temperature sensor", this->name_.c_str());
    return -1.0;
  }
  return this->temperature_ges_->getTemperature();
}

IMotionCubeState Joint::getIMotionCubeState()
{
  IMotionCubeState states;

  std::bitset<16> statusWordBits = this->imc_->getStatusWord();
  states.statusWord = statusWordBits.to_string();
  std::bitset<16> motionErrorBits = this->imc_->getMotionError();
  states.motionError = motionErrorBits.to_string();
  std::bitset<16> detailedErrorBits = this->imc_->getDetailedError();
  states.detailedError = detailedErrorBits.to_string();
  std::bitset<16> secondDetailedErrorBits = this->imc_->getSecondDetailedError();
  states.secondDetailedError = secondDetailedErrorBits.to_string();

  states.state = IMCState(this->imc_->getStatusWord());

  states.motionErrorDescription = error::parseError(this->imc_->getMotionError(), error::ErrorRegisters::MOTION_ERROR);
  states.detailedErrorDescription =
      error::parseError(this->imc_->getDetailedError(), error::ErrorRegisters::DETAILED_ERROR);
  states.secondDetailedErrorDescription =
      error::parseError(this->imc_->getSecondDetailedError(), error::ErrorRegisters::SECOND_DETAILED_ERROR);

  states.motorCurrent = this->imc_->getMotorCurrent();
  states.IMCVoltage = this->imc_->getIMCVoltage();
  states.motorVoltage = this->imc_->getMotorVoltage();

  states.absoluteEncoderValue = this->imc_->getAngleIUAbsolute();
  states.incrementalEncoderValue = this->imc_->getAngleIUIncremental();
  states.absoluteVelocity = this->imc_->getVelocityIUAbsolute();
  states.incrementalVelocity = this->imc_->getVelocityIUIncremental();

  return states;
}

void Joint::setAllowActuation(bool allow_actuation)
{
  this->allow_actuation_ = allow_actuation;
}

int Joint::getTemperatureGESSlaveIndex() const
{
  if (this->hasTemperatureGES())
  {
    return this->temperature_ges_->getSlaveIndex();
  }
  return -1;
}

int Joint::getIMotionCubeSlaveIndex() const
{
  if (this->hasIMotionCube())
  {
    return this->imc_->getSlaveIndex();
  }
  return -1;
}

int Joint::getNetNumber() const
{
  return this->net_number_;
}

std::string Joint::getName() const
{
  return this->name_;
}

bool Joint::hasIMotionCube() const
{
  return this->imc_ != nullptr;
}

bool Joint::hasTemperatureGES() const
{
  return this->temperature_ges_ != nullptr;
}

bool Joint::canActuate() const
{
  return this->allow_actuation_ && this->hasIMotionCube();
}

bool Joint::receivedDataUpdate()
{
  if (!this->hasIMotionCube())
  {
    return false;
  }
  // If imc voltage, motor current, and both encoders positions and velocities did not change,
  // we probably did not receive an update for this joint.
  float new_imc_volt = this->imc_->getIMCVoltage();
  float new_motor_volt = this->imc_->getMotorVoltage();
  float new_motor_current = this->imc_->getMotorCurrent();
  double new_absolute_position = this->imc_->getAngleRadAbsolute();
  double new_incremental_position = this->imc_->getAngleRadIncremental();
  double new_absolute_velocity = this->imc_->getVelocityRadAbsolute();
  double new_incremental_velocity = this->imc_->getVelocityRadIncremental();
  bool data_updated = (new_imc_volt != this->previous_imc_volt_ || new_motor_volt != this->previous_motor_volt_ ||
                       new_motor_current != this->previous_motor_current_ ||
                       new_absolute_position != this->previous_absolute_position_ ||
                       new_incremental_position != this->previous_incremental_position_ ||
                       new_absolute_velocity != this->previous_absolute_velocity_ ||
                       new_incremental_velocity != this->previous_incremental_velocity_);

  this->previous_imc_volt_ = new_imc_volt;
  this->previous_motor_volt_ = new_motor_volt;
  this->previous_motor_current_ = new_motor_current;
  this->previous_absolute_position_ = new_absolute_position;
  this->previous_incremental_position_ = new_incremental_position;
  this->previous_absolute_velocity_ = new_absolute_velocity;
  this->previous_incremental_velocity_ = new_incremental_velocity;
  return data_updated;
}

ActuationMode Joint::getActuationMode() const
{
  return this->imc_->getActuationMode();
}

}  // namespace march
