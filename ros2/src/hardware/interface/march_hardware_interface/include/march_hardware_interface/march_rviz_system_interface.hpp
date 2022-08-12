//
// Created by george on 13-6-22.
//

#ifndef MARCH_HARDWARE_INTERFACE__MARCH_RVIZ_SYSTEM_INTERFACE_HPP_
#define MARCH_HARDWARE_INTERFACE__MARCH_RVIZ_SYSTEM_INTERFACE_HPP_

#include <hardware_interface/types/hardware_interface_type_values.hpp>
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
#include "march_hardware_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "iostream"

namespace march_hardware_interface {
class MarchRvizSystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarchRvizSystemInterface);

    MARCH_HARDWARE_INTERFACE_PUBLIC
    MarchRvizSystemInterface();

    MARCH_HARDWARE_INTERFACE_PUBLIC ~MarchRvizSystemInterface();

    MARCH_HARDWARE_INTERFACE_PUBLIC
    ~MarchRvizSystemInterface() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type start() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type stop() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write() override;

private:

    const std::shared_ptr<rclcpp::Logger> logger_;
    static const std::string COMMAND_AND_STATE_TYPE; // = hardware_interface::HW_IF_POSITION
    std::vector<double> hw_positions_;
    double pdb_current_;


};

} // namespace march_hardware_interface

#endif // MARCH_HARDWARE_INTERFACE__MARCH_RVIZ_SYSTEM_INTERFACE_HPP_
