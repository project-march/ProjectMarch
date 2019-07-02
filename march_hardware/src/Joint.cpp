// Copyright 2019 Project March.
#include <ros/ros.h>

#include <bitset>

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

std::vector<std::string> Joint::getIMotionCubeState()
{
    // Return a vector of strings:
    // First the literal bits of the Status Word, Detailed Error and Motion Error
    // Then the parsed interpretation of these bits
    std::vector<std::string> return_strings;

    std::bitset<16> statusWordBits = this->iMotionCube.getStatusWord();
    return_strings.push_back(statusWordBits.to_string());
    std::bitset<16> detailedErrorBits = this->iMotionCube.getDetailedError();
    return_strings.push_back(detailedErrorBits.to_string());
    std::bitset<16> motionErrorBits = this->iMotionCube.getMotionError();
    return_strings.push_back(motionErrorBits.to_string());

    return_strings.push_back(this->iMotionCube.getState(this->iMotionCube.getStatusWord()));
    return_strings.push_back(this->iMotionCube.parseDetailedError(this->iMotionCube.getDetailedError()));
    return_strings.push_back(this->iMotionCube.parseMotionError(this->iMotionCube.getMotionError()));

    return return_strings;
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
