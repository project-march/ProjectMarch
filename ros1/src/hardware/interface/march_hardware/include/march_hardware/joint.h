// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_JOINT_H
#define MARCH_HARDWARE_JOINT_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <march_hardware/motor_controller/motor_controller.h>
#include <march_hardware/power/power_distribution_board.h>
#include <march_hardware/temperature/temperature_ges.h>
#include <march_hardware/motor_controller/motor_controller_state.h>

namespace march
{
class Joint
{
public:
  // Initialize a Joint with motor controller and without temperature slave.
  Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<MotorController> motor_controller);

  // Initialize a Joint with motor controller and temperature slave.
  Joint(std::string name, int net_number, bool allow_actuation, std::unique_ptr<MotorController> motor_controller,
        std::unique_ptr<TemperatureGES> temperature_ges);

  virtual ~Joint() noexcept = default;

  // Delete move assignment since string cannot be move assigned
  Joint(Joint&&) = default;
  Joint& operator=(Joint&&) = delete;

  // Call the initSdo functions of the components of the joint
  bool initSdo(int cycle_time);

  // Read the encoder data and store the position and velocity values in the Joint
  void readEncoders(const ros::Duration& elapsed_time);

  // Check whether the state of the MotorController has changed
  bool receivedDataUpdate();

  // Prepare the joint for actuation
  // First calls the prepareActuation() method of the MotorController
  // Then sets some initial values
  void prepareActuation();

  // Actuate the joint if it is allowed to do so
  void actuate(double target);

  // Get the position and velocity of the joint
  double getPosition() const;
  double getVelocity() const;

  // Getters and setters for properties of the joint
  std::string getName() const;
  int getNetNumber() const;
  bool canActuate() const;
  void setAllowActuation(bool allow_actuation);

  // A joint must have a MotorController
  std::unique_ptr<MotorController>& getMotorController();

  // A joint may have a temperature GES
  bool hasTemperatureGES() const;
  std::unique_ptr<TemperatureGES>& getTemperatureGES();

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
       << "MotorController: ";
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

  // Keep track of the position and velocity of the joint, updated by readEncoders()
  double previous_incremental_position_ = 0.0;
  double position_ = 0.0;
  double velocity_ = 0.0;

  // Keep track of the state of the MotorController
  std::unique_ptr<MotorControllerState> previous_state_ = nullptr;

  // A joint must have a MotorController but may have a TemperatureGES
  std::unique_ptr<MotorController> motor_controller_;
  std::unique_ptr<TemperatureGES> temperature_ges_ = nullptr;
};

}  // namespace march
#endif  // MARCH_HARDWARE_JOINT_H
