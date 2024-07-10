// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_H
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include "march_hardware/torque_sensor/torque_sensor.h"
#include <chrono>
#include <memory>
#include <string>

namespace march {
class MotorController : public Slave {
public:
    MotorController(const Slave& slave, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        std::unique_ptr<IncrementalEncoder> incremental_encoder, std::unique_ptr<TorqueSensor> torque_sensor,
        ActuationMode actuation_mode, bool use_inc_enc_for_position, std::shared_ptr<march_logger::BaseLogger> logger);

    // Get the most precise position or velocity
    float getPosition();
    float getVelocity();

    // Get the position, either absolute or incremental
    // Will throw an HardwareException if the MotorController doesn't have the
    // requested encoder
    float getAbsolutePosition();
    float getAbsoluteVelocity();
    float getIncrementalPosition();
    float getIncrementalVelocity();
    float getTorque();

    /**
     * \brief Applies an clamp, an motor-direction and Effort to IU Effort conversion transformation.
     * @param joint_effort_command The general effort command.
     * @return The effort ready to be sent to the controller.
     */
    double getMotorControllerSpecificEffort(double joint_effort_command) const;

    // A MotorController should support both actuating by position (radians) or
    // by torque
    virtual void actuateTorque(float target_effort, float fuzzy_weight) = 0;
    virtual void actuateRadians(float target_position, float fuzzy_weight) = 0;

    virtual void sendTargetPosition(float target_position) = 0;
    virtual void sendPID(std::array<double, 3> pos_pid, std::array<double, 2> tor_pid) = 0;

    // Getter and setter for the ActuationMode
    ActuationMode getActuationMode() const;
    void setActuationMode(ActuationMode actuation_mode);

    /* Reset the MotorController
     * Can be overridden by child classes
     * @returns Returns an optional wait duration
     */
    virtual std::chrono::nanoseconds reset();

    // Prepare the MotorController for actuation
    // Returns an optional wait duration
    virtual std::chrono::nanoseconds prepareActuation() = 0;

    // Enable actuation for the MotorController, move it into its 'ready' state
    // Returns an optional wait duration
    virtual void enableActuation() = 0;

    // Transform the ActuationMode to a number that is understood by the
    // MotorController
    virtual int getActuationModeNumber() const = 0;

     // Are the slaves of this MotorController unique
    virtual bool requiresUniqueSlaves() const = 0;

    // Check if we want to use the incremental encoder to update the position
    bool useIncrementalEncoderForPosition() const;

    // A MotorController doesn't necessarily have an AbsoluteEncoder and an
    // IncrementalEncoder, but will have at least one of the two
    bool hasAbsoluteEncoder() const;
    std::unique_ptr<AbsoluteEncoder>& getAbsoluteEncoder();
    bool hasIncrementalEncoder() const;
    std::unique_ptr<IncrementalEncoder>& getIncrementalEncoder();

    bool hasTorqueSensor() const;
    std::unique_ptr<TorqueSensor>& getTorqueSensor();

    // Getters for specific information about the state of the motor and its
    // controller
    // virtual float getTorque() = 0;
    virtual float getMotorCurrent() = 0;
    virtual float getMotorControllerVoltage() = 0;
    virtual float getMotorVoltage() = 0;
    virtual float getActualEffort() = 0;

    // Get a full description of the state of the MotorController
    virtual std::unique_ptr<MotorControllerState> getState() = 0;

    // Effort may have to be multiplied by a constant
    // because ROS control limits the pid values to a certain maximum
    virtual double effortMultiplicationConstant() const;

    // Get the effort limit of the motor controller
    virtual double getTorqueLimit() const = 0;

    ~MotorController() override = default;

    // Override comparison operator
    friend bool operator==(const MotorController& lhs, const MotorController& rhs)
    {
        return lhs.getSlaveIndex() == rhs.getSlaveIndex()
            && ((lhs.absolute_encoder_ && rhs.absolute_encoder_ && *lhs.absolute_encoder_ == *rhs.absolute_encoder_)
                || (!lhs.absolute_encoder_ && !rhs.absolute_encoder_))
            && ((lhs.incremental_encoder_ && rhs.incremental_encoder_
                    && *lhs.incremental_encoder_ == *rhs.incremental_encoder_)
                || (!lhs.incremental_encoder_ && !rhs.incremental_encoder_))
            && lhs.actuation_mode_.getValue() == rhs.actuation_mode_.getValue();
    }
    // Override stream operator for clean printing
    friend std::ostream& operator<<(std::ostream& os, const MotorController& motor_controller)
    {
        os << "slave index: " << motor_controller.getSlaveIndex();

        if (motor_controller.hasAbsoluteEncoder()) {
            os << ", absolute encoder: " << *motor_controller.absolute_encoder_;
        }
        if (motor_controller.hasIncrementalEncoder()) {
            os << ", incremental encoder: " << *motor_controller.incremental_encoder_;
        }
        os << ", actuation mode" << motor_controller.actuation_mode_.toString();
        return os;
    }

    // Watchdog base time = 1 / 25 MHz * (2498 + 2) = 0.0001 seconds=100 µs
    static const uint16_t WATCHDOG_DIVIDER = 2498;
    // 500 * 100us = 50 ms = watchdog timer
    static const uint16_t WATCHDOG_TIME = 500;

protected:
    /// Get the direction of the most significant encoder.
    Encoder::Direction getMotorDirection() const;

    // Getters for absolute and incremental position and velocity.
    // These will not check whether the encoder actually exists but may give a
    // segmentation fault.
    virtual float getAbsolutePositionUnchecked() = 0;
    virtual float getIncrementalPositionUnchecked() = 0;
    virtual float getAbsoluteVelocityUnchecked() = 0;
    virtual float getIncrementalVelocityUnchecked() = 0;
    virtual float getTorqueUnchecked() = 0;

    std::unique_ptr<AbsoluteEncoder> absolute_encoder_;
    std::unique_ptr<IncrementalEncoder> incremental_encoder_;
    std::unique_ptr<TorqueSensor> torque_sensor_;
    ActuationMode actuation_mode_;
    bool use_inc_enc_for_position_;

    std::shared_ptr<march_logger::BaseLogger> logger_;

private:
    /**
     * \brief This converts the effort from amper to the Internal Units the motor controller uses.
     * \note This conversion rate is dictated by the `effortMultiplicationConstant` of the motor controller. When:
     * `this->effortMultiplicationConstant()` = 1 => Internal Units = Ampere.
     * @param joint_effort_command The joint effort command in Ampere.
     * @return The joint effort command in IU.
     */
    double convertEffortToIUEffort(double joint_effort_command) const;
};

} // namespace march
#endif // MARCH_HARDWARE_MOTOR_CONTROLLER_H
