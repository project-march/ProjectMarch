// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H

#include <array>
#include <optional>
#include <string>

namespace march {
class MotorControllerState {
public:
    MotorControllerState() = default;

    double motor_current_ = 0;
    double motor_voltage_ = 0;
    double motor_controller_voltage_ = 0;
    double motor_temperature_ = 0;
    double motor_controller_temperature_ = 0;
    double absolute_position_iu_ = 0;
    double incremental_position_iu_ = 0;
    double absolute_velocity_iu_ = 0;
    double incremental_velocity_iu_ = 0;
    double absolute_position_ = 0;
    double incremental_position_ = 0;
    double absolute_velocity_ = 0;
    double incremental_velocity_ = 0;
    double AIE_absolute_position_ = 0;
    double torque_ = 0;

    friend bool operator==(const MotorControllerState& lhs, const MotorControllerState& rhs)
    {
        return lhs.motor_current_ == rhs.motor_current_ && lhs.motor_voltage_ == rhs.motor_voltage_
            && lhs.motor_controller_voltage_ == rhs.motor_controller_voltage_
            && lhs.motor_temperature_ == rhs.motor_temperature_
            && lhs.motor_controller_temperature_ == rhs.motor_controller_temperature_
            && lhs.absolute_position_iu_ == rhs.absolute_position_iu_
            && lhs.incremental_position_iu_ == rhs.incremental_position_iu_
            && lhs.absolute_velocity_iu_ == rhs.absolute_velocity_iu_
            && lhs.incremental_velocity_iu_ == rhs.incremental_velocity_iu_
            && lhs.absolute_position_ == rhs.absolute_position_
            && lhs.incremental_position_ == rhs.incremental_position_
            && lhs.absolute_velocity_ == rhs.absolute_velocity_
            && lhs.incremental_velocity_ == rhs.incremental_velocity_ 
            && lhs.AIE_absolute_position_ == rhs.AIE_absolute_position_
            && lhs.torque_ == rhs.torque_
            && lhs.isOperational() == rhs.isOperational()
            && lhs.hasError() == rhs.hasError();
    }

    /**
     * Check whether the motor controller data is valid
     * @details Used by the joint to verify that etherCAT connection has been
     * made
     * @return true if the motor controller data is valid
     */
    virtual bool dataIsValid() const = 0;

    /**
     * Check whether the motor controller is in state 8 (closed loop control), and can be actively actuated.
     * @return true if the motor controller is in an operational state,
     * otherwise false
     */
    virtual bool isOperational() const = 0;

    /**
     * Check whether the motor controller has an error
     * @return true if the motor controller has an error, otherwise false
     */
    virtual bool hasError() const = 0;

    /**
     * Get a string description of the state and error states of the motor
     * controller
     * @return string describing the current state as well as the error state(s)
     * of the motor controller
     */
    virtual std::string getErrorStatus() const = 0;

    /**
     * Get a string description of the operational state
     * @return string describing the operational state
     */
    virtual std::string getOperationalState() const = 0;

    /**
     * @brief Gives a pointer to the member variables so that the hardware interface can couple it back to the
     * broadcaster.
     * @attention Should contain exactly the same interface_names as expected by the
     * `march_motor_controller_state_broadcaster/.../motor_controller_semantic_component.hpp`
     * @return A pair of interface_names and pointers to the member variables.
     */
    inline std::array<std::pair<std::string, double*>, 14> get_pointers()
    {
        return {
            std::make_pair(/*__x=*/"motor_current", &motor_current_),
            std::make_pair(/*__x=*/"motor_voltage", &motor_voltage_),
            std::make_pair(/*__x=*/"motor_temperature", &motor_temperature_),
            std::make_pair(/*__x=*/"motor_controller_temperature", &motor_controller_temperature_),
            std::make_pair(/*__x=*/"absolute_position_iu", &absolute_position_iu_),
            std::make_pair(/*__x=*/"incremental_position_iu", &incremental_position_iu_),
            std::make_pair(/*__x=*/"absolute_velocity_iu", &absolute_velocity_iu_),
            std::make_pair(/*__x=*/"incremental_velocity_iu", &incremental_velocity_iu_),
            std::make_pair(/*__x=*/"absolute_position", &absolute_position_),
            std::make_pair(/*__x=*/"incremental_position", &incremental_position_),
            std::make_pair(/*__x=*/"absolute_velocity", &absolute_velocity_),
            std::make_pair(/*__x=*/"incremental_velocity", &incremental_velocity_),
            std::make_pair(/*__x=*/"AIE_absolute_position", &AIE_absolute_position_),
            std::make_pair(/*__x=*/"torque", &torque_),
        };
    }

    /**
     * @brief Updates the member variables based on the input array.
     * @param other The other motor controller state of which all values need to be copied over from.
     */
    inline void update_values(MotorControllerState* other)
    {
        motor_current_ = other->motor_current_;
        motor_voltage_ = other->motor_voltage_;
        motor_temperature_ = other->motor_temperature_;
        motor_controller_temperature_ = other->motor_controller_temperature_;
        absolute_position_iu_ = other->absolute_position_iu_;
        incremental_position_iu_ = other->incremental_position_iu_;
        absolute_velocity_iu_ = other->absolute_velocity_iu_;
        incremental_velocity_iu_ = other->incremental_velocity_iu_;
        absolute_position_ = other->absolute_position_;
        incremental_position_ = other->incremental_position_;
        absolute_velocity_ = other->absolute_velocity_;
        incremental_velocity_ = other->incremental_velocity_;
        AIE_absolute_position_ = other->AIE_absolute_position_;
        torque_ = other->torque_;
    }
};

} // namespace march

#endif // MARCH_HARDWARE_MOTOR_CONTROLLER_STATES_H
