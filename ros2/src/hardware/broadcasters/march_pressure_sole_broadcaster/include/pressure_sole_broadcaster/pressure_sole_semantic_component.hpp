/// @author Marco Bak - M8

#ifndef BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
#define BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP

#include <limits>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "march_shared_msgs/msg/pressure_soles_data.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace march_pressure_sole_broadcaster {
using StateInerfaces = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
using PressureSolesMsg = march_shared_msgs::msg::PressureSolesData;

namespace state_interface {
    using StateInerfaces = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
    inline double get(int& index, const StateInerfaces& state_interfaces)
    {
        index++;
        double ret = state_interfaces[index-1].get().get_value();
        RCLCPP_INFO(rclcpp::get_logger("get_state_interfaces_loggger"), "index state_interface value is: %f", ret);
        return ret;
    }
} // namespace state_interface

class PressureSoleSemanticComponent : public semantic_components::SemanticComponentInterface<PressureSolesMsg> {
public:
    explicit PressureSoleSemanticComponent()
        : SemanticComponentInterface("pressure_soles", /*size*/ 2)
    {
        interface_names_.emplace_back(name_ + "/" + "heel_left");
        interface_names_.emplace_back(name_ + "/" + "heel_right");
        // interface_names_.emplace_back(name_ + "/" + "l_met1");
        // interface_names_.emplace_back(name_ + "/" + "l_hallux");
        // interface_names_.emplace_back(name_ + "/" + "l_met3");
        // interface_names_.emplace_back(name_ + "/" + "l_toes");
        // interface_names_.emplace_back(name_ + "/" + "l_met5");
        // interface_names_.emplace_back(name_ + "/" + "l_arch");
        // interface_names_.emplace_back(name_ + "/" + "r_heel_right");
        // interface_names_.emplace_back(name_ + "/" + "r_heel_left");
        // interface_names_.emplace_back(name_ + "/" + "r_met1");
        // interface_names_.emplace_back(name_ + "/" + "r_hallux");
        // interface_names_.emplace_back(name_ + "/" + "r_met3");
        // interface_names_.emplace_back(name_ + "/" + "r_toes");
        // interface_names_.emplace_back(name_ + "/" + "r_met5");
        // interface_names_.emplace_back(name_ + "/" + "r_arch");
    }

    virtual ~PressureSoleSemanticComponent() = default;

    /** @brief Updates the local variables with the values retrieved from the borrowed state interface.
     *  @attention Make sure this happens in the same order as they are placed in the constructor, and the same names
     *  are defined in `march_hardware/.../pressure_sole.h::get_pointers()`.
     *
     *  How this method works is that within the march_hardware_interface state interfaces are registered.
     *  These state interfaces contain a pointer to a member variable of type double.
     *  This pointer can be retrieved via the `this->state_interfaces_` but in the order that they are assigned in
     *  `this->interface_names_`. So we use the method `state_interface::get(index, ...)` method which updates the index
     *  by one, and returns the value.
     */
    void update()
    {

        // RCLCPP_INFO(rclcpp::get_logger("test_loggger"), "Updating values");
        int index = 0;
        // RCLCPP_INFO(rclcpp::get_logger("test_loggger"), "Updating values");
        heel_left_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        heel_right_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // l_met1_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // l_hallux_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // l_met3_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // l_toes_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // l_met5_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // l_arch_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_heel_right_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_heel_left_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_met1_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_hallux_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_met3_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_toes_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_met5_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        // r_arch_ = static_cast<float>(state_interface::get(index, state_interfaces_));

        
    }

    bool get_values_as_message(PressureSolesMsg& msg)
    {
        update();
        // RCLCPP_INFO(rclcpp::get_logger("test_loggger"), "l_heel_left pressure is: %f", l_heel_left_);
        msg.names = { "l_heel_right", "l_heel_left", "l_met1", "l_hallux", "l_met3", "l_toes", "l_met5", "l_arch",
            "r_heel_right", "r_heel_left", "r_met1", "r_hallux", "r_met3", "r_toes", "r_met5", "r_arch" };
        msg.pressure_values = { heel_right_, heel_left_, l_met1_, l_hallux_, l_met3_, l_toes_, l_met5_, l_arch_,
            r_heel_right_, r_heel_left_, r_met1_, r_hallux_, r_met3_, r_toes_, r_met5_, r_arch_ };
        return true;
    }

private:
    float heel_right_ = 2.f;
    float heel_left_ = 0.f;
    float l_met1_ = 0.f;
    float l_hallux_ = 0.f;
    float l_met3_ = 0.f;
    float l_toes_ = 0.f;
    float l_met5_ = 0.f;
    float l_arch_ = 0.f;
    float r_heel_right_ = 0.f;
    float r_heel_left_ = 0.f;
    float r_met1_ = 0.f;
    float r_hallux_ = 0.f;
    float r_met3_ = 0.f;
    float r_toes_ = 0.f;
    float r_met5_ = 0.f;
    float r_arch_ = 0.f;
};
} // namespace march_pressure_sole_broadcaster

#endif // BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
