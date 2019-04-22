// Copyright 2019 Project March.
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
  bool allowActuation;
  IMotionCube iMotionCube;
  TemperatureGES temperatureGES;

public:
  // TODO(Tim) pass by reference or pointer instead of making copy
  Joint(std::string name, bool actuate, TemperatureGES temperatureGES, IMotionCube iMotionCube);
  Joint(std::string name, bool actuate, TemperatureGES temperatureGES);
  Joint(std::string name, bool actuate, IMotionCube iMotionCube);

  void initialize(int ecatCycleTime);
  void actuateRad(float targetPositionRad);

  float getAngleRad();
  float getTemperature();

  std::string getName();
  int getTemperatureGESSlaveIndex();
  int getIMotionCubeSlaveIndex();
  IMotionCube getIMotionCube();

  bool hasIMotionCube();
  bool hasTemperatureGES();
  bool canActuate();

  /** @brief Override comparison operator */
  friend bool operator==(const Joint& lhs, const Joint& rhs)
  {
    return lhs.name == rhs.name && lhs.iMotionCube == rhs.iMotionCube && lhs.temperatureGES == rhs.temperatureGES &&
           lhs.allowActuation == rhs.allowActuation;
  }

  friend bool operator!=(const Joint& lhs, const Joint& rhs)
  {
    return !(lhs == rhs);
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
  {
    return os << "name: " << joint.name << ", "
              << "actuate: " << joint.allowActuation<< ", "
              << "imotioncube: " << joint.iMotionCube << ","
              << "temperatureges: " << joint.temperatureGES;
  }
};
}  // namespace march4cpp
#endif
