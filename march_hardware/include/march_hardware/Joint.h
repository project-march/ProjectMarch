// Copyright 2019 Project March.
#ifndef MARCH4CPP__JOINT_H
#define MARCH4CPP__JOINT_H

#include <string>
#include <vector>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <march_hardware/TemperatureGES.h>
#include <march_hardware/IMotionCubeState.h>

namespace march4cpp
{

class Joint
{
private:
  std::string name;
  // Set this number via the hardware builder
  int netNumber;
  bool allowActuation;
  IMotionCube iMotionCube;
  TemperatureGES temperatureGES;

public:
  Joint(): name(""), netNumber(-1), allowActuation(false)
  {
  }

  void initialize(int ecatCycleTime);
  void prepareActuation();
  // TODO(Martijn) Refactor this to make joint less dependent on knowledge of the IMC
  void resetIMotionCube();

  void actuateRad(float targetPositionRad);
  void actuateTorque(int targetTorque);

  float getAngleRad();
  float getTorque();
  int getAngleIU();
  float getTemperature();
  IMotionCubeState getIMotionCubeState();

  std::string getName();
  int getTemperatureGESSlaveIndex();
  int getIMotionCubeSlaveIndex();
  int getNetNumber()
  {
    return netNumber;
  }

  ActuationMode getActuationMode() const;

  void setActuationMode(ActuationMode actuationMode);

  bool hasIMotionCube();
  bool hasTemperatureGES();
  bool canActuate();

  /** @brief Override comparison operator */
  friend bool operator==(const Joint& lhs, const Joint& rhs)
  {
    return lhs.name == rhs.name && lhs.iMotionCube == rhs.iMotionCube && lhs.temperatureGES == rhs.temperatureGES &&
           lhs.allowActuation == rhs.allowActuation &&
           lhs.getActuationMode().getValue() == rhs.getActuationMode().getValue();
  }

  friend bool operator!=(const Joint& lhs, const Joint& rhs)
  {
    return !(lhs == rhs);
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
  {
    return os << "name: " << joint.name << ", "
              << "ActuationMode: " << joint.getActuationMode().toString() << ", "
              << "allowActuation: " << joint.allowActuation << ", "
              << "imotioncube: " << joint.iMotionCube << ","
              << "temperatureges: " << joint.temperatureGES;
  }

  void setName(const std::string& name);
  void setAllowActuation(bool allowActuation);
  void setIMotionCube(const IMotionCube& iMotionCube);
  void setTemperatureGes(const TemperatureGES& temperatureGes);
  void setNetNumber(int netNumber);
};

}  // namespace march4cpp
#endif
