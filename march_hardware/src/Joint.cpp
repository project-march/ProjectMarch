#include <utility>

#include <utility>

#include <stdlib.h>
#include <math.h>
#include <stdexcept>
#include "ros/ros.h"
#include <march_hardware/Joint.h>

Joint::Joint(std::string name, TemperatureSensor* temperatureSensor, IMotionCube* iMotionCube)
{
  this->name = std::move(name);
  this->temperatureSensor = temperatureSensor;
  this->iMotionCube = iMotionCube;
}

void Joint::actuate(double position)
{
  // TODO(Martijn) write to ethercat
  // TODO(BaCo) check that the position is allowed and does not exceed (torque) limits.
}

float Joint::getTemperature()
{
  if (this->temperatureSensor == NULL)
  {
    ROS_WARN("Joint %s has no temperaturesensor", this->name.c_str());
    return -1;
  }
  return this->temperatureSensor->getTemperature();
}

float Joint::getAngle()
{
  if (this->iMotionCube == NULL)
  {
    ROS_WARN("Joint %s has no iMotionCube", this->name.c_str());
    return -1;
  }
  return this->iMotionCube->getAngle();
}

std::string Joint::getName()
{
  return this->name;
}

void Joint::initialize()
{
  if (this->iMotionCube != NULL)
  {
    iMotionCube->initialize();
  }
  if (this->temperatureSensor != NULL)
  {
    temperatureSensor->initialize();
  }
}