/// @author Marco Bak - M8

#ifndef BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
#define BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP

#include <limits>
#include <string>
#include <vector>

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
        return state_interfaces[index - 1].get().get_value();
    }
} // namespace state_interface

class PressureSoleSemanticComponent : public semantic_components::SemanticComponentInterface<PressureSolesMsg> {
public:
    explicit PressureSoleSemanticComponent()
        : SemanticComponentInterface("pressure_soles", /*size*/ 16)
    {
        interface_names_.emplace_back(name_ + "/" + "l_heel_right");
        interface_names_.emplace_back(name_ + "/" + "l_heel_left");
        interface_names_.emplace_back(name_ + "/" + "l_met1");
        interface_names_.emplace_back(name_ + "/" + "l_hallux");
        interface_names_.emplace_back(name_ + "/" + "l_met3");
        interface_names_.emplace_back(name_ + "/" + "l_toes");
        interface_names_.emplace_back(name_ + "/" + "l_met5");
        interface_names_.emplace_back(name_ + "/" + "l_arch");
        interface_names_.emplace_back(name_ + "/" + "r_heel_right");
        interface_names_.emplace_back(name_ + "/" + "r_heel_left");
        interface_names_.emplace_back(name_ + "/" + "r_met1");
        interface_names_.emplace_back(name_ + "/" + "r_hallux");
        interface_names_.emplace_back(name_ + "/" + "r_met3");
        interface_names_.emplace_back(name_ + "/" + "r_toes");
        interface_names_.emplace_back(name_ + "/" + "r_met5");
        interface_names_.emplace_back(name_ + "/" + "r_arch");
    }

    virtual ~PressureSoleSemanticComponent() = default;

    void update()
    {
        int index = 0;
        l_heel_left_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        l_heel_right_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        l_met1_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        l_hallux_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        l_met3_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        l_toes_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        l_met5_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        l_arch_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_heel_left_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_heel_left_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_met1_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_hallux_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_met3_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_toes_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_met5_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        r_arch_ = static_cast<float>(state_interface::get(index, state_interfaces_));
    }

    bool get_values_as_message(PressureSolesMsg& msg)
    {
        update();
        msg.l_heel_left = l_heel_left_;
        msg.l_heel_right = l_heel_right_;
        msg.l_met1 = l_met1_;
        msg.l_hallux = l_hallux_;
        msg.l_met3 = l_met3_;
        msg.l_toes = l_toes_;
        msg.l_met5 = l_met5_;
        msg.l_arch = l_arch_;
        msg.r_heel_left = r_heel_left_;
        msg.r_heel_right = r_heel_right_;
        msg.r_met1 = r_met1_;
        msg.r_hallux = r_hallux_;
        msg.r_met3 = r_met3_;
        msg.r_toes = r_toes_;
        msg.r_met5 = r_met5_;
        msg.r_arch = r_arch_;
        return true;
    }

private:
    float l_heel_right_ = 0.F;
    float l_heel_left_ = 0.F;
    float l_met1_ = 0.F;
    float l_hallux_ = 0.F;
    float l_met3_ = 0.F;
    float l_toes_ = 0.F;
    float l_met5_ = 0.F;
    float l_arch_ = 0.F;
    float r_heel_right_ = 0.F;
    float r_heel_left_ = 0.F;
    float r_met1_ = 0.F;
    float r_hallux_ = 0.F;
    float r_met3_ = 0.F;
    float r_toes_ = 0.F;
    float r_met5_ = 0.F;
    float r_arch_ = 0.F;
};
} // namespace march_pressure_sole_broadcaster

#endif // BUILD_PRESSURE_SOLE_SEMANTIC_COMPONENT_HPP
