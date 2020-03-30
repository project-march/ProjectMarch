// Copyright 2019 Project March.
#include <ros/ros.h>

#include <bitset>
#include <memory>
#include <string>
#include <utility>

#include <march_hardware/error/motion_error.h>
#include <march_hardware/Joint.h>
#include <march_hardware/error/hardware_exception.h>

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

void Joint::initialize(int cycle_time)
{
  if (this->hasIMotionCube())
  {
    this->imc_->writeInitialSDOs(cycle_time);
  }
  if (this->hasTemperatureGES())
  {
    this->temperature_ges_->writeInitialSDOs(cycle_time);
  }
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
}

void Joint::resetIMotionCube()
{
  if (hasIMotionCube())
  {
    this->imc_->resetIMotionCube();
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

double Joint::getAngleRadAbsolute()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getAngleRadAbsolute();
}

double Joint::getAngleRadIncremental()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getAngleRadIncremental();
}

double Joint::getAngleRadMostPrecise()
{
  if (!this->hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name_.c_str());
    return -1;
  }
  return this->imc_->getAngleRadMostPrecise();
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
  std::bitset<16> detailedErrorBits = this->imc_->getDetailedError();
  states.detailedError = detailedErrorBits.to_string();
  std::bitset<16> motionErrorBits = this->imc_->getMotionError();
  states.motionError = motionErrorBits.to_string();

  states.state = IMCState(this->imc_->getStatusWord());
  states.detailedErrorDescription = error::parseDetailedError(this->imc_->getDetailedError());
  states.motionErrorDescription = error::parseMotionError(this->imc_->getMotionError());

  states.motorCurrent = this->imc_->getMotorCurrent();
  states.motorVoltage = this->imc_->getMotorVoltage();

  states.absoluteEncoderValue = this->imc_->getAngleIUAbsolute();
  states.incrementalEncoderValue = this->imc_->getAngleIUIncremental();

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
  return this->imc_ && this->imc_->getSlaveIndex() != -1;
}

bool Joint::hasTemperatureGES() const
{
  return this->temperature_ges_ && this->temperature_ges_->getSlaveIndex() != -1;
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
  // We assume that the motor voltage cannot remain precisely constant.
  float new_motor_volt = this->imc_->getMotorVoltage();
  bool data_updated = (new_motor_volt != this->previous_motor_volt_);
  this->previous_motor_volt_ = new_motor_volt;
  return data_updated;
}

ActuationMode Joint::getActuationMode() const
{
  return this->imc_->getActuationMode();
}

}  // namespace march
