//
// Created by george on 13-6-22.
//

#ifndef MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
#define MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "march_hardware_interface/visibility_control.h"

namespace march_hardware_interface
{
class MarchExoSystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarchExoSystemInterface);

    MARCH_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type start() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type stop() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type read() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
            hardware_interface::return_type write() override;

private:
    // Parameters for the RRBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;

    // Store the command for the simulated robot
    std::vector<double> hw_position_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
};

}  // namespace march_hardware_interface

#endif // MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
