// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H

#include <optional>
#include <string>

namespace march {
class MotorControllerState {
public:
    MotorControllerState() = default;

    float motor_current_ {};
    float motor_voltage_ {};
    float motor_controller_voltage_ {};
    float absolute_position_iu_ {};
    float incremental_position_iu_ {};
    float absolute_velocity_iu_ {};
    float incremental_velocity_iu_ {};
    float absolute_position_ {};
    float incremental_position_ {};
    float absolute_velocity_ {};
    float incremental_velocity_ {};

    friend bool operator==(
        const MotorControllerState& lhs, const MotorControllerState& rhs)
    {
        return lhs.motor_current_ == rhs.motor_current_
            && lhs.motor_voltage_ == rhs.motor_voltage_
            && lhs.absolute_position_iu_ == rhs.absolute_position_iu_
            && lhs.incremental_position_iu_ == rhs.incremental_position_iu_
            && lhs.absolute_velocity_iu_ == rhs.absolute_velocity_iu_
            && lhs.incremental_velocity_iu_ == rhs.incremental_velocity_iu_
            && lhs.absolute_position_ == rhs.absolute_position_
            && lhs.incremental_position_ == rhs.incremental_position_
            && lhs.absolute_velocity_ == rhs.absolute_velocity_
            && lhs.incremental_velocity_ == rhs.incremental_velocity_;
    }

    /**
     * Check whether the motor controller is in an operational state
     * @return true if the motor controller is in an operational state,
     * otherwise false
     */
    virtual bool isOperational() = 0;

    /**
     * Check whether the motor controller has an error
     * @return true if the motor controller has an error, otherwise false
     */
    virtual bool hasError() = 0;

    /**
     * Get a string description of the state and error states of the motor
     * controller
     * @return string describing the current state as well as the error state(s)
     * of the motor controller
     */
    virtual std::optional<std::string> getErrorStatus() = 0;

    /**
     * Get a string description of the operational state
     * @return string describing the operational state
     */
    virtual std::string getOperationalState() = 0;
};

} // namespace march

#endif // MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
