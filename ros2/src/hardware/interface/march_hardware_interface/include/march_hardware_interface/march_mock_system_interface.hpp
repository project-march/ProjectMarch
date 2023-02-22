//
// Created by george on 13-6-22.
//

#ifndef MARCH_HARDWARE_INTERFACE__MARCH_MOCK_SYSTEM_INTERFACE_HPP_
#define MARCH_HARDWARE_INTERFACE__MARCH_MOCK_SYSTEM_INTERFACE_HPP_

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <march_hardware/motor_controller/odrive/odrive_state.h>
#include <march_hardware/power_distribution_board/power_distribution_board.h>
#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "march_hardware_interface/march_mock_system_interface.hpp"
#include "march_hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"

namespace march_hardware_interface {

/// Contains all the needed information for the Hardware Interface.
struct HwStateInfo {
    std::string name;
    double hw_position;
    double hw_velocity;
    double hw_effort;
};

class MarchMockSystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarchMockSystemInterface);

    MARCH_HARDWARE_INTERFACE_PUBLIC
    MarchMockSystemInterface();

    MARCH_HARDWARE_INTERFACE_PUBLIC
    ~MarchMockSystemInterface() override;

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type start() override;

    hardware_interface::return_type stop() override;

    hardware_interface::return_type read() override;

    hardware_interface::return_type write() override;

private:
    const std::shared_ptr<rclcpp::Logger> logger_;
    static const std::string COMMAND_AND_STATE_TYPE; // = hardware_interface::HW_IF_POSITION
    std::vector<HwStateInfo> hw_state_info_;
    std::vector<double> hw_positions_;

    march::PowerDistributionBoardData pdb_data_;
    std::vector<march::PressureSoleData> pressure_soles_data_;
    std::vector<march::ODriveState> motor_controllers_data_;
};

} // namespace march_hardware_interface

#endif // MARCH_HARDWARE_INTERFACE__MARCH_MOCK_SYSTEM_INTERFACE_HPP_
