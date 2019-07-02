// Copyright 2019 Project March.
#include <ros/ros.h>

#include <march_hardware/Joint.h>

namespace march4cpp
{
Joint::Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, IMotionCube iMotionCube)
  : temperatureGES(temperatureGES), iMotionCube(iMotionCube)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}
Joint::Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES, IMotionCube iMotionCube,
             int netNumber)
  : temperatureGES(temperatureGES), iMotionCube(iMotionCube), netNumber(netNumber)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

Joint::Joint(std::string name, bool allowActuation, TemperatureGES temperatureGES) : temperatureGES(temperatureGES)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}
Joint::Joint(std::string name, bool allowActuation, IMotionCube iMotionCube) : iMotionCube(iMotionCube)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

Joint::Joint(std::string name, bool allowActuation, IMotionCube iMotionCube, int netNumber)
  : iMotionCube(iMotionCube), netNumber(netNumber)
{
  this->name = std::move(name);
  this->allowActuation = allowActuation;
}

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
  if (this->allowActuation)
  {
    this->iMotionCube.goToOperationEnabled();
  }
  else
  {
    ROS_ERROR("Trying to prepare joint %s for actuation while it is not "
              "allowed to actuate",
              this->name.c_str());
  }
}

void Joint::actuateRad(float targetPositionRad)
{
  ROS_ASSERT_MSG(this->allowActuation, "Joint %s is not allowed to actuate, "
                                       "yet its actuate method has been "
                                       "called.",
                 this->name.c_str());
  // TODO(BaCo) check that the position is allowed and does not exceed (torque)
  // limits.
  this->iMotionCube.actuateRad(targetPositionRad);
}

float Joint::getAngleRad()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("Joint %s has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube.getAngleRad();
}

float Joint::getTemperature()
{
  if (!hasTemperatureGES())
  {
    ROS_WARN("Joint %s has no temperature sensor", this->name.c_str());
    return -1;
  }
  return this->temperatureGES.getTemperature();
}

std::vector<uint16> Joint::getIMotionCubeErrorState()
{
    std::vector<uint16> errors;
    errors.push_back(this->iMotionCube.getStatusWord());
    errors.push_back(this->iMotionCube.getDetailedError());
    errors.push_back(this->iMotionCube.getMotionError());
    return errors;
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
}  // namespace march4cpp
