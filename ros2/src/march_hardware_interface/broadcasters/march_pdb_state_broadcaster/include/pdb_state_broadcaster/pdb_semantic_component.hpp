/// @author George Vegelien - M7

#ifndef MARCH_PDB_STATE_BROADCASTER__PDB_SEMANTIC_COMPONENT_HPP_
#define MARCH_PDB_STATE_BROADCASTER__PDB_SEMANTIC_COMPONENT_HPP_

#include <limits>
#include <string>
#include <vector>

#include "march_shared_msgs/msg/power_distribution_board_data.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace march_pdb_state_broadcaster {
using StateInerfaces = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
using HighVoltageMsg = march_shared_msgs::msg::HighVoltageState;

namespace state_interface {
    inline double get(int& index, const StateInerfaces& state_interfaces)
    {
        index++;
        return state_interfaces[index - 1].get().get_value();
    }
} // namespace state_interface

class HighVoltageState {
private:
    float total_current_ = 0.F;
    std::array<float, 4> hv_currents_ {};

public:
    void update(int& index, const StateInerfaces& state_interfaces)
    {
        total_current_ = static_cast<float>(state_interface::get(index, state_interfaces));
        for (auto& hv_current : hv_currents_) {
            hv_current = static_cast<float>(state_interface::get(index, state_interfaces));
        }
    }

    explicit operator HighVoltageMsg() const
    {
        march_shared_msgs::msg::HighVoltageState hv_msg {};
        hv_msg.total_current = total_current_;
        hv_msg.hv1_current = hv_currents_[0];
        hv_msg.hv2_current = hv_currents_[1];
        hv_msg.hv3_current = hv_currents_[2];
        hv_msg.hv4_current = hv_currents_[3];
        return hv_msg;
    }
};

using LowVoltageMsg = march_shared_msgs::msg::LowVoltageState;

class LowVoltageState {
private:
    std::array<float, 2> lv_currents_ {};
    std::array<bool, 2> are_lvs_ok_ {};

public:
    void update(int& index, const StateInerfaces& state_interfaces)
    {
        for (auto& lv_current : lv_currents_) {
            lv_current = static_cast<float>(state_interface::get(index, state_interfaces));
        }
        for (auto& is_lv_ok : are_lvs_ok_) {
            is_lv_ok = static_cast<bool>(state_interface::get(index, state_interfaces));
        }
    }

    explicit operator LowVoltageMsg() const
    {
        march_shared_msgs::msg::LowVoltageState lv_msg {};
        lv_msg.lv1_current = lv_currents_[0];
        lv_msg.lv2_current = lv_currents_[1];
        lv_msg.lv1_ok = are_lvs_ok_[0];
        lv_msg.lv2_ok = are_lvs_ok_[1];
        return lv_msg;
    }
};

using BatteryMsg = march_shared_msgs::msg::BatteryState;

class BatteryState {
private:
    float battery_percentage_ = 0.F;
    float battery_voltage_ = 0.F;
    float battery_temperature_ = 0.F;

public:
    void update(int& index, const StateInerfaces& state_interfaces)
    {
        battery_percentage_ = static_cast<float>(state_interface::get(index, state_interfaces));
        battery_voltage_ = static_cast<float>(state_interface::get(index, state_interfaces));
        battery_temperature_ = static_cast<float>(state_interface::get(index, state_interfaces));
    }

    explicit operator BatteryMsg() const
    {
        march_shared_msgs::msg::BatteryState battery_msg {};
        battery_msg.battery_percentage = battery_percentage_;
        battery_msg.battery_voltage = battery_voltage_;
        battery_msg.battery_temperature = battery_temperature_;
        return battery_msg;
    }
};

using PdbMsg = march_shared_msgs::msg::PowerDistributionBoardData;

class PdbSemanticComponent : public semantic_components::SemanticComponentInterface<PdbMsg> {
public:
    explicit PdbSemanticComponent()
        : SemanticComponentInterface("PDB", /*size=*/15)
    {
        interface_names_.emplace_back(name_ + "/" + "pdb_current");
        interface_names_.emplace_back(name_ + "/" + "hv_state.total_current");
        interface_names_.emplace_back(name_ + "/" + "hv_state.hv1_current");
        interface_names_.emplace_back(name_ + "/" + "hv_state.hv2_current");
        interface_names_.emplace_back(name_ + "/" + "hv_state.hv3_current");
        interface_names_.emplace_back(name_ + "/" + "hv_state.hv4_current");
        interface_names_.emplace_back(name_ + "/" + "lv_state.lv1_current");
        interface_names_.emplace_back(name_ + "/" + "lv_state.lv2_current");
        interface_names_.emplace_back(name_ + "/" + "lv_state.lv1_ok");
        interface_names_.emplace_back(name_ + "/" + "lv_state.lv2_ok");
        interface_names_.emplace_back(name_ + "/" + "battery_state.percentage");
        interface_names_.emplace_back(name_ + "/" + "battery_state.voltage");
        interface_names_.emplace_back(name_ + "/" + "battery_state.temperature");
        interface_names_.emplace_back(name_ + "/" + "pdb_temperature");
        interface_names_.emplace_back(name_ + "/" + "checksum_miso");
    }

    virtual ~PdbSemanticComponent() = default;

    /** @brief Updates the local variables with the values retrieved from the borrowed state interface.
     *  @attention Make sure this happens in the same order as they are placed in the constructor, and the same names
     *  are defined in `march_hardware/.../power_distribution_board.h::get_pointers()`.
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
        pdb_current_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        hv_state_.update(index, state_interfaces_);
        lv_state_.update(index, state_interfaces_);
        battery_state_.update(index, state_interfaces_);
        pdb_temperature_ = static_cast<float>(state_interface::get(index, state_interfaces_));
        checksum_miso_ = static_cast<float>(state_interface::get(index, state_interfaces_));
    }

    /**
     * @brief Fills the pdb message with updated values.
     *
     * @param msg The reference to the pdb message that we fill up.
     * @return True to overwrite the unimplemented default.
     */
    bool get_values_as_message(PdbMsg& msg)
    {
        // update with the latest values
        update();
        msg.pdb_current = pdb_current_;
        msg.hv_state = static_cast<HighVoltageMsg>(hv_state_);
        msg.lv_state = static_cast<LowVoltageMsg>(lv_state_);
        msg.battery_state = static_cast<BatteryMsg>(battery_state_);
        msg.pdb_temperature = pdb_temperature_;
        msg.checksum_miso = checksum_miso_;
        return true;
    }

private:
    float pdb_current_ = 0.F;
    HighVoltageState hv_state_ {};
    LowVoltageState lv_state_ {};
    BatteryState battery_state_ {};
    float pdb_temperature_ = 0.F;
    float checksum_miso_ = 0.F;
};

} // namespace march_pdb_state_broadcaster

#endif // MARCH_PDB_STATE_BROADCASTER__PDB_SEMANTIC_COMPONENT_HPP_
