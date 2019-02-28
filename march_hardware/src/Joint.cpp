#include <ros/ros.h>

#include <march_hardware/Joint.h>

namespace march4cpp
{
Joint::Joint(std::string name, TemperatureSensor temperatureSensor, IMotionCube iMotionCube)
  : temperatureSensor(temperatureSensor), iMotionCube(iMotionCube)
{
  this->name = std::move(name);
}

Joint::Joint(std::string name, TemperatureSensor temperatureSensor) : temperatureSensor(temperatureSensor)
{
  this->name = std::move(name);
}
Joint::Joint(std::string name, IMotionCube iMotionCube) : iMotionCube(iMotionCube)

{
  this->name = std::move(name);
}

void Joint::initialize()
{
  if (hasIMotionCube())
  {
    iMotionCube.initialize();
  }
  if (hasTemperatureSensor())
  {
    temperatureSensor.initialize();
  }
}

void Joint::actuateRad(float targetPositionRad)
{
  // TODO(BaCo) check that the position is allowed and does not exceed (torque) limits.
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
  if (!hasTemperatureSensor())
  {
    ROS_WARN("Joint %s has no temperature sensor", this->name.c_str());
    return -1;
  }
  return this->temperatureSensor.getTemperature();
}

int Joint::getTemperatureSensorSlaveIndex()
{
  if (hasTemperatureSensor())
  {
    return this->temperatureSensor.getSlaveIndex();
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

IMotionCube Joint::getIMotionCube()
{
  return this->iMotionCube;
}

std::string Joint::getName()
{
  return this->name;
}

bool Joint::hasIMotionCube()
{
  return this->iMotionCube.getSlaveIndex() != -1;
}

bool Joint::hasTemperatureSensor()
{
  return this->temperatureSensor.getSlaveIndex() != -1;
}
}