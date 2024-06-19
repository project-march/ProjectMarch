/// @author George Vegelien - M7

#ifndef MARCH_MOTOR_CONTROLLER_STATE_BROADCASTER__MOTOR_CONTROLLER_SEMANTIC_COMPONENT_HPP_
#define MARCH_MOTOR_CONTROLLER_STATE_BROADCASTER__MOTOR_CONTROLLER_SEMANTIC_COMPONENT_HPP_

#include <limits>
#include <string>
#include <vector>

#include "march_shared_msgs/msg/joint_motor_controller_state.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace march_motor_controller_state_broadcaster {
namespace state_interface {
    using StateInerfaces = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
    inline double get(int& index, const StateInerfaces& state_interfaces)
    {
        index++;
        return state_interfaces[index - 1].get().get_value();
    }
} // namespace state_interface

using JointMCMsg = march_shared_msgs::msg::JointMotorControllerState;

class MotorControllerSemanticComponent : public semantic_components::SemanticComponentInterface<JointMCMsg> {
public:
    explicit MotorControllerSemanticComponent(const std::string& joint_name)
        : SemanticComponentInterface(joint_name, /*size=*/14)
        : SemanticComponentInterface(joint_name, /*size=*/14)
        , joint_name_(joint_name)
    {
        interface_names_.emplace_back(joint_name + "/" + "motor_current");
        interface_names_.emplace_back(joint_name + "/" + "motor_voltage");
        interface_names_.emplace_back(joint_name + "/" + "motor_temperature");
        interface_names_.emplace_back(joint_name + "/" + "motor_controller_temperature");
        interface_names_.emplace_back(joint_name + "/" + "absolute_position_iu");
        interface_names_.emplace_back(joint_name + "/" + "incremental_position_iu");
        interface_names_.emplace_back(joint_name + "/" + "absolute_velocity_iu");
        interface_names_.emplace_back(joint_name + "/" + "incremental_velocity_iu");
        interface_names_.emplace_back(joint_name + "/" + "absolute_position");
        interface_names_.emplace_back(joint_name + "/" + "incremental_position");
        interface_names_.emplace_back(joint_name + "/" + "absolute_velocity");
        interface_names_.emplace_back(joint_name + "/" + "incremental_velocity");
        interface_names_.emplace_back(joint_name + "/" + "AIE_absolute_position");
        interface_names_.emplace_back(joint_name + "/" + "torque");
        interface_names_.emplace_back(joint_name + "/" + "check_sum");
    }

    virtual ~MotorControllerSemanticComponent() = default;

    /** @brief Updates the local variables with the values retrieved from the borrowed state interface.
     *  @attention Make sure this happens in the same order as they are placed in the constructor, and the same names
     *  are defined in `march_hardware/.../motor_controller_state.h::get_pointers()`.
     *
     *  How this method works is that within the march_system_interface state interfaces are registered.
     *  These state interfaces contain a pointer to a member variable of type double.
     *  This pointer can be retrieved via the `this->state_interfaces_` but in the order that they are assigned in
     *  `this->interface_names_`. So we use the method `state_interface::get(index, ...)` method which updates the index
     *  by one, and returns the value.
     */
    void update()
    {
        int index = 0;
        motor_current_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        motor_voltage_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        motor_temperature_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        motor_controller_temperature_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        absolute_position_iu_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        incremental_position_iu_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        absolute_velocity_iu_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        incremental_velocity_iu_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        absolute_position_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        incremental_position_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        absolute_velocity_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        incremental_velocity_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        AIE_absolute_position_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        torque_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        check_sum_ = static_cast<float>(state_interface::get(index, state_interfaces_));
    }

    /**
     * @brief Fills the pdb message with updated values.
     *
     * @param msg The reference to the pdb message that we fill up.
     * @return True to overwrite the unimplemented default.
     */
    bool get_values_as_message(JointMCMsg& msg)
    {
        // update with the latest values
        update();
        msg.joint_name = joint_name_;
        msg.motor_current = motor_current_;
        msg.motor_voltage = motor_voltage_;
        msg.motor_temperature = motor_temperature_;
        msg.motor_controller_temperature = motor_controller_temperature_;
        msg.absolute_position_iu = absolute_position_iu_;
        msg.incremental_position_iu = incremental_position_iu_;
        msg.absolute_velocity_iu = absolute_velocity_iu_;
        msg.incremental_velocity_iu = incremental_velocity_iu_;
        msg.absolute_position = absolute_position_;
        msg.incremental_position = incremental_position_;
        msg.absolute_velocity = absolute_velocity_;
        msg.incremental_velocity = incremental_velocity_;
        msg.aie_absolute_position = AIE_absolute_position_;
        msg.torque = torque_;
        msg.check_sum = check_sum_;
        return true;
    }

private:
    std::string joint_name_;

    float motor_current_ = 0.F;
    float motor_voltage_ = 0.F;
    float motor_temperature_ = 0.F;
    float motor_controller_temperature_ = 0.F;

    float absolute_position_iu_ = 0.F;
    float incremental_position_iu_ = 0.F;
    float absolute_velocity_iu_ = 0.F;
    float incremental_velocity_iu_ = 0.F;

    float absolute_position_ = 0.F;
    float incremental_position_ = 0.F;
    float absolute_velocity_ = 0.F;
    float incremental_velocity_ = 0.F;

    float AIE_absolute_position_ = 0.F;
    float torque_ = 0.F;
    float check_sum_ = 0.F;
};

} // namespace march_motor_controller_state_broadcaster

#endif // MARCH_MOTOR_CONTROLLER_STATE_BROADCASTER__MOTOR_CONTROLLER_SEMANTIC_COMPONENT_HPP_
