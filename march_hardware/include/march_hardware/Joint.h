#ifndef MARCH4CPP__JOINT_H
#define MARCH4CPP__JOINT_H

#include <string>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/TemperatureSensor.h>
namespace march4cpp
{
class Joint
{
private:
  std::string name;
  IMotionCube* iMotionCube;
  TemperatureSensor* temperatureSensor;

public:
  Joint(std::string name, TemperatureSensor* temperatureSensor = NULL, IMotionCube* iMotionCube = NULL);

  void initialize();
  void actuate(double effort);

  float getAngle();
  float getTemperature();

  std::string getName();
  int getTemperatureSensorSlaveIndex();
  int getIMotionCubeSlaveIndex();

  bool hasIMotionCube();
  bool hasTemperatureSensor();
};
}  // namespace march4cpp
#endif
