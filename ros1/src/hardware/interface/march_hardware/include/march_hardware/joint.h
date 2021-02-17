// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_JOINT_H
#define MARCH_HARDWARE_JOINT_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <march_hardware/motor_controller/imotioncube/imotioncube.h>
#include <march_hardware/power/power_distribution_board.h>
#include <march_hardware/temperature/temperature_ges.h>
#include <march_hardware/motor_controller/motor_controller_state.h>

namespace march
{
class Joint
{
public:
  /**
   * Initializes a Joint without motor controller and temperature slave.
   * Actuation will be disabled.
   */
//  Joint(std::string name, int net_number);

  /**
   * Initializes a Joint with motor controller and without temperature slave.
   */
  Joint(std::string name, int net_number, bool allow_actuation, std::shared_ptr<IMotionCube> motor_controller);

  /**
   * Initializes a Joint with motor controller and temperature slave.
   */
  Joint(std::string name, int net_number, bool allow_actuation, std::shared_ptr<IMotionCube> motor_controller,
        std::shared_ptr<TemperatureGES> temperature_ges);

  virtual ~Joint() noexcept = default;

  /* Delete copy constructor/assignment since the unique_ptr cannot be copied */
  Joint(const Joint&) = delete;
  Joint& operator=(const Joint&) = delete;

  /* Delete move assignment since string cannot be move assigned */
  Joint(Joint&&) = default;
  Joint& operator=(Joint&&) = delete;

  // Initialize the components of the joint
  bool initSdo(int cycle_time);
  void prepareActuation();

  void actuate(double target);
  void readEncoders(const ros::Duration& elapsed_time);

  double getPosition() const;
  double getVelocity() const;

  std::string getName() const;
  int getNetNumber() const;

  std::shared_ptr<MotorController> getMotorController();

  bool hasTemperatureGES() const;
  std::shared_ptr<TemperatureGES> getTemperatureGES();

  bool canActuate() const;
  bool receivedDataUpdate();
  void setAllowActuation(bool allow_actuation);

  /** @brief Override comparison operator */
  friend bool operator==(const Joint& lhs, const Joint& rhs)
  {
    return lhs.name_ == rhs.name_ &&
           ((lhs.motor_controller_ && rhs.motor_controller_ && *lhs.motor_controller_ == *rhs.motor_controller_) ||
            (!lhs.motor_controller_ && !rhs.motor_controller_)) &&
           ((lhs.temperature_ges_ && rhs.temperature_ges_ && *lhs.temperature_ges_ == *rhs.temperature_ges_) ||
            (!lhs.temperature_ges_ && !rhs.temperature_ges_)) &&
           lhs.allow_actuation_ == rhs.allow_actuation_;
  }

  friend bool operator!=(const Joint& lhs, const Joint& rhs)
  {
    return !(lhs == rhs);
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
  {
    os << "name: " << joint.name_ << ", "
       << "allowActuation: " << joint.allow_actuation_ << ", "
       << "imotioncube: ";
    if (joint.motor_controller_)
    {
      os << *joint.motor_controller_;
    }
    else
    {
      os << "none";
    }

    os << ", temperatureges: ";
    if (joint.temperature_ges_)
    {
      os << *joint.temperature_ges_;
    }
    else
    {
      os << "none";
    }

    return os;
  }

private:
  const std::string name_;
  const int net_number_;
  bool allow_actuation_ = false;

  std::unique_ptr<MotorControllerState> previous_state_ = nullptr;

  double previous_incremental_position_ = 0.0;
  double position_ = 0.0;
  double velocity_ = 0.0;

  std::shared_ptr<MotorController> motor_controller_;
  std::shared_ptr<TemperatureGES> temperature_ges_ = nullptr;
};

}  // namespace march
#endif  // MARCH_HARDWARE_JOINT_H
