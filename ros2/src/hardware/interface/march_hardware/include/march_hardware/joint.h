// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_JOINT_H
#define MARCH_HARDWARE_JOINT_H

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <march_hardware/motor_controller/motor_controller.h>
#include <march_hardware/motor_controller/motor_controller_state.h>
#include <march_hardware/temperature/temperature_ges.h>

namespace march {
class Joint {
public:
    // Initialize a Joint with motor controller and without temperature slave.
    // MotorController cannot be a nullptr, since a Joint should always have a
    // MotorController.
    Joint(std::string name, int net_number, std::unique_ptr<MotorController> motor_controller,
        std::unique_ptr<std::array<double, 3>> position_pid, std::unique_ptr<std::array<double, 3>> torque_pid,
        std::shared_ptr<march_logger::BaseLogger> logger);

    // Initialize a Joint with motor controller and temperature slave.
    // MotorController cannot be a nullptr, since a Joint should always have a
    // MotorController. OdriveTemperature ges may be a nullptr, since a Joint may have
    // a OdriveTemperature ges.
    Joint(std::string name, int net_number, std::unique_ptr<MotorController> motor_controller,
        std::unique_ptr<std::array<double, 3>> position_pid, std::unique_ptr<std::array<double, 3>> torque_pid,
        std::unique_ptr<TemperatureGES> temperature_ges, std::shared_ptr<march_logger::BaseLogger> logger);

    virtual ~Joint() noexcept = default;

    // Delete move assignment since string cannot be move assigned
    Joint(Joint&&) = default;
    Joint& operator=(Joint&&) = delete;

    // Call the initSdo functions of the components of the joint
    bool initSdo(int cycle_time);

    // Read the encoder data and store the position and velocity values in the
    // Joint
    void readEncoders();

    // Function to send the PID values to the ODrives for the joint.
    void sendPID();

    // Check whether the state of the MotorController has changed
    bool receivedDataUpdate();

    // Prepare the joint for actuation
    // Returns an optional wait duration
    std::chrono::nanoseconds prepareActuation();

    // Enable actuation for this joint
    // Returns an optional wait duration
    void enableActuation();

    // Set initial encoder values
    void readFirstEncoderValues(bool operational_check);

    // Actuate the joint
    void actuate(float target_position, float target_torque, float position_weight, float torque_weight);

    // Get the position, velocity and torque of the joint
    double getPosition() const;
    double getVelocity() const;
    double getTorque() const;

    // Getters and setters for properties of the joint
    std::string getName() const;
    int getNetNumber() const;
    bool canActuate() const;

    // A joint must have a MotorController
    std::unique_ptr<MotorController>& getMotorController();

    // A joint may have a temperature GES
    bool hasTemperatureGES() const;
    std::unique_ptr<TemperatureGES>& getTemperatureGES();

    /**
     * \brief Checks whether the joint is in its soft limits.
     * \note It uses old position data. Joint::readEncoders(...) must be called before hand.
     * @return True if it is in an 'oke' state.
     */
    bool isWithinSoftLimits() const;

    /**
     * \brief Checks whether the joint is in its error soft limits.
     * \note It uses old position data. Joint::readEncoders(...) must be called before hand.
     * @return True if it is in an 'oke' state.
     */
    bool isWithinSoftErrorLimits() const;

    /**
     * \brief Checks whether the joint is in its hard limits.
     * \note It uses old position data. Joint::readEncoders(...) must be called before hand.
     * @return True if it is in an 'oke' state.
     */
    bool isWithinHardLimits() const;

    /** @brief Override comparison operator */
    friend bool operator==(const Joint& lhs, const Joint& rhs)
    {
        return lhs.name_ == rhs.name_
            && ((lhs.motor_controller_ && rhs.motor_controller_ && *lhs.motor_controller_ == *rhs.motor_controller_)
                || (!lhs.motor_controller_ && !rhs.motor_controller_))
            && ((lhs.temperature_ges_ && rhs.temperature_ges_ && *lhs.temperature_ges_ == *rhs.temperature_ges_)
                || (!lhs.temperature_ges_ && !rhs.temperature_ges_));
    }

    friend bool operator!=(const Joint& lhs, const Joint& rhs)
    {
        return !(lhs == rhs);
    }
    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(std::ostream& os, const Joint& joint)
    {
        os << "name: " << joint.name_ << ", "
           << "MotorController: " << *joint.motor_controller_;
        os << ", temperatureges: ";
        if (joint.hasTemperatureGES()) {
            os << *joint.temperature_ges_;
        } else {
            os << "none";
        }
        return os;
    }

private:
    const std::string name_;
    const int net_number_;

    // Keep track of the position and velocity of the joint, updated by
    // readEncoders()
    double initial_incremental_position_ = 0.0;
    double initial_absolute_position_ = 0.0;
    double initial_torque_ = 0.0f;
    double position_ = 0.0; // In radians
    double velocity_ = 0.0;
    double torque_ = 0.0f;
    std::chrono::steady_clock::time_point last_read_time_;

    // Keep track of the state of the MotorController
    std::optional<std::unique_ptr<MotorControllerState>> previous_state_ = std::nullopt;

    // A joint must have a MotorController but may have a TemperatureGES
    std::unique_ptr<MotorController> motor_controller_;

    // Array with the position PID and torque PID values of the joint
    std::unique_ptr<std::array<double, 3>> position_pid;
    std::unique_ptr<std::array<double, 3>> torque_pid;

    std::unique_ptr<TemperatureGES> temperature_ges_ = nullptr;
    std::shared_ptr<march_logger::BaseLogger> logger_;
};

} // namespace march
#endif // MARCH_HARDWARE_JOINT_H
