#include <ros/ros.h>

#include <march_hardware/Joint.h>

namespace march4cpp
{

Joint::Joint(std::string name, TemperatureSensor* temperatureSensor, IMotionCube* iMotionCube)
{
  this->name = std::move(name);
  this->temperatureSensor = temperatureSensor;
  this->iMotionCube = iMotionCube;
}

void Joint::initialize()
{
  if (hasIMotionCube())
  {
    iMotionCube->initialize();
  }
  if (hasTemperatureSensor())
  {
    temperatureSensor->initialize();
  }
}

void Joint::actuate(double position)
{
  // TODO(Martijn) write to ethercat
  // TODO(BaCo) check that the position is allowed and does not exceed (torque) limits.
}

float Joint::getAngle()
{
  if (!hasIMotionCube())
  {
    ROS_WARN("Joint %s has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube->getAngle();
}

float Joint::getTemperature()
{
  if (!hasTemperatureSensor())
  {
    ROS_WARN("Joint %s has no temperaturesensor", this->name.c_str());
    return -1;
  }
  return this->temperatureSensor->getTemperature();
}

int Joint::getTemperatureSensorSlaveIndex()
{
  if (hasTemperatureSensor())
  {
    return this->temperatureSensor->getSlaveIndex();
  }
  return -1;
}

int Joint::getIMotionCubeSlaveIndex()
{
  if (hasIMotionCube())
  {
    return this->iMotionCube->getSlaveIndex();
  }
  return -1;
}

std::string Joint::getName()
{
  return this->name;
}

bool Joint::hasIMotionCube()
{
  return iMotionCube != NULL;
}

bool Joint::hasTemperatureSensor()
{
  return temperatureSensor != NULL;
}
}