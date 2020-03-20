// Copyright 2019 Project March.
#include <ros/ros.h>

#include <bitset>
#include <string>

#include <march_hardware/error/motion_error.h>
#include <march_hardware/Joint.h>
#include <march_hardware/error/hardware_exception.h>

namespace march
{
void Joint::initialize(int ecatCycleTime)
{
  if (hasIMotionCube())
  {
    iMotionCube.writeInitialSDOs(ecatCycleTime);
  }
  if (hasTemperatureGES())
  {
    temperatureGES.writeInitialSDOs(ecatCycleTime);
  }
}

void Joint::prepareActuation()
{
  if (!this->allowActuation)
  {
    throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE, "Failed to prepare joint %s for actuation",
                                   this->name.c_str());
  }
  ROS_INFO("[%s] Preparing for actuation", this->name.c_str());
  this->iMotionCube.goToOperationEnabled();
  ROS_INFO("[%s] Successfully prepared for actuation", this->name.c_str());
}

void Joint::actuateRad(double targetPositionRad)
{
  if (!this->allowActuation)
  {
    throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE, "Joint %s is not allowed to actuate",
                                   this->name.c_str());
  }
  this->iMotionCube.actuateRad(targetPositionRad);
}

double Joint::getAngleRadAbsolute()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleRadAbsolute();
}

double Joint::getAngleRadIncremental()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleRadIncremental();
}

double Joint::getAngleRadMostPrecise()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleRadMostPrecise();
}

void Joint::actuateTorque(int16_t targetTorque)
{
  if (!this->allowActuation)
  {
    throw error::HardwareException(error::ErrorType::NOT_ALLOWED_TO_ACTUATE, "Joint %s is not allowed to actuate",
                                   this->name.c_str());
  }
  this->iMotionCube.actuateTorque(targetTorque);
}

int16_t Joint::getTorque()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getTorque();
}

int32_t Joint::getAngleIUAbsolute()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleIUAbsolute();
}

int32_t Joint::getAngleIUIncremental()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("[%s] Has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleIUIncremental();
}

float Joint::getTemperature()
{
  if (!hasTemperatureGES())
  {
    ROS_WARN("[%s] Has no temperature sensor", this->name.c_str());
    return -1;
  }
  return this->temperatureGES.getTemperature();
}

IMotionCubeState Joint::getIMotionCubeState()
{
  IMotionCubeState states;

  std::bitset<16> statusWordBits = this->iMotionCube.getStatusWord();
  states.statusWord = statusWordBits.to_string();
  std::bitset<16> detailedErrorBits = this->iMotionCube.getDetailedError();
  states.detailedError = detailedErrorBits.to_string();
  std::bitset<16> motionErrorBits = this->iMotionCube.getMotionError();
  states.motionError = motionErrorBits.to_string();

  states.state = IMCState(this->iMotionCube.getStatusWord());
  states.detailedErrorDescription = error::parseDetailedError(this->iMotionCube.getDetailedError());
  states.motionErrorDescription = error::parseMotionError(this->iMotionCube.getMotionError());

  states.motorCurrent = this->iMotionCube.getMotorCurrent();
  states.motorVoltage = this->iMotionCube.getMotorVoltage();

  states.incrementalEncoderValue = this->iMotionCube.getAngleIUIncremental();

  return states;
}

void Joint::setName(const std::string& name)
{
  Joint::name = name;
}
void Joint::setAllowActuation(bool allowActuation)
{
  Joint::allowActuation = allowActuation;
}

void Joint::setTemperatureGes(const TemperatureGES& temperatureGes)
{
  temperatureGES = temperatureGes;
}

int Joint::getTemperatureGESSlaveIndex()
{
  if (hasTemperatureGES())
  {
    return this->temperatureGES.getSlaveIndex();
  }
  return -1;
}

int Joint::getIMotionCubeSlaveIndex()
{
  if (hasIMotionCube())
  {
    return this->iMotionCube.getSlaveIndex();
  }
  return -1;
}
std::string Joint::getName()
{
  return this->name;
}

bool Joint::hasIMotionCube()
{
  return this->iMotionCube.getSlaveIndex() != -1;
}

bool Joint::hasTemperatureGES()
{
  return this->temperatureGES.getSlaveIndex() != -1;
}

bool Joint::canActuate()
{
  return this->allowActuation;
}

void Joint::setNetNumber(int netNumber)
{
  Joint::netNumber = netNumber;
}

ActuationMode Joint::getActuationMode() const
{
  return this->iMotionCube.getActuationMode();
}

}  // namespace march
