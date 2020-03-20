// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_JOINT_H
#define MARCH_HARDWARE_JOINT_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <march_hardware/IMotionCube.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <march_hardware/TemperatureGES.h>
#include <march_hardware/IMotionCubeState.h>

namespace march
{
class Joint
{
public:
  /**
   * Initializes a Joint without motor controller and temperature slave.
   * Actuation will be disabled.
   */
  Joint(std::string name, int net_number);

  /**
   * Initializes a Joint with motor controller and without temperature slave.
   */
  Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<IMotionCube> imc);

  /**
   * Initializes a Joint with motor controller and temperature slave.
   */
  Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<IMotionCube> imc,
        std::unique_ptr<TemperatureGES> temperature_ges);

  virtual ~Joint() noexcept = default;

  /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;

  void initialize(int cycle_time);
  void prepareActuation();

  void actuateRad(double target_position);
  void actuateTorque(int16_t target_torque);

  double getAngleRadAbsolute();
  double getAngleRadIncremental();
  double getAngleRadMostPrecise();
  int16_t getTorque();
  int32_t getAngleIUAbsolute();
  int32_t getAngleIUIncremental();
  float getTemperature();
  IMotionCubeState getIMotionCubeState();

  std::string getName() const;
  int getTemperatureGESSlaveIndex() const;
  int getIMotionCubeSlaveIndex() const;
  int getNetNumber() const;

  ActuationMode getActuationMode() const;

  bool hasIMotionCube() const;
  bool hasTemperatureGES() const;
  bool canActuate() const;
  void setAllowActuation(bool allow_actuation);

  /** @brief Override comparison operator */
  friend bool operator==(const Joint& lhs, const Joint& rhs)
  {
    return lhs.name_ == rhs.name_ && lhs.imc_ == rhs.imc_ && lhs.temperature_ges_ == rhs.temperature_ges_ &&
           lhs.allow_actuation_ == rhs.allow_actuation_ &&
           lhs.getActuationMode().getValue() == rhs.getActuationMode().getValue();
  }

  friend bool operator!=(const Joint& lhs, const Joint& rhs)
  {
    return !(lhs == rhs);
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
  {
    return os << "name: " << joint.name_ << ", "
              << "ActuationMode: " << joint.getActuationMode().toString() << ", "
              << "allowActuation: " << joint.allow_actuation_ << ", "
              << "imotioncube: " << joint.imc_.get() << ","
              << "temperatureges: " << joint.temperature_ges_.get();
  }

private:
  const std::string name_;
  const int net_number_;
  bool allow_actuation_ = false;

  std::unique_ptr<IMotionCube> imc_ = nullptr;
  std::unique_ptr<TemperatureGES> temperature_ges_ = nullptr;
};

}  // namespace march
#endif  // MARCH_HARDWARE_JOINT_H
