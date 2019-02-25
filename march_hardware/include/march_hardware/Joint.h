#ifndef MARCH4CPP__JOINT_H
#define MARCH4CPP__JOINT_H

#include <string>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/TemperatureGES.h>
namespace march4cpp
{
class Joint
{
private:
  std::string name;
  IMotionCube iMotionCube;
  TemperatureGES* temperatureGES;

public:
  Joint(std::string name, TemperatureGES* temperatureGES, IMotionCube iMotionCube);
  Joint(std::string name, TemperatureGES* temperatureGES);
  Joint(std::string name, IMotionCube iMotionCube);

  void initialize();
  void actuate(double effort);

  float getAngle();
  float getTemperature();

  std::string getName();
  int getTemperatureGESSlaveIndex();
  int getIMotionCubeSlaveIndex();

  bool hasIMotionCube();
  bool hasTemperatureGES();
};
}  // namespace march4cpp
#endif
