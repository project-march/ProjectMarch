// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_JOINT_H
#define MARCH_HARDWARE_JOINT_H

#include <string>
#include <vector>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <march_hardware/TemperatureGES.h>
#include <march_hardware/IMotionCubeState.h>

namespace march
{
class Joint
{
private:
  std::string name;
  // Set this number via the hardware builder
  int netNumber;
  bool allowActuation;
  float previous_motor_volt_ = 0.0;
  IMotionCube iMotionCube;
  TemperatureGES temperatureGES;

public:
  explicit Joint(const IMotionCube& imc) : name(""), netNumber(-1), allowActuation(false), iMotionCube(imc)
  {
  }

  void initialize(int ecatCycleTime);
  void prepareActuation();
  void shutdown();

  void actuateRad(double targetPositionRad);
  void actuateTorque(int16_t targetTorque);

  double getAngleRadAbsolute();
  double getAngleRadIncremental();
  double getAngleRadMostPrecise();
  int16_t getTorque();
  int32_t getAngleIUAbsolute();
  int32_t getAngleIUIncremental();
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

  bool hasIMotionCube();
  bool hasTemperatureGES();
  bool canActuate();
  bool receivedDataUpdate();

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
  void setTemperatureGes(const TemperatureGES& temperatureGes);
  void setNetNumber(int netNumber);
};

}  // namespace march
#endif  // MARCH_HARDWARE_JOINT_H
