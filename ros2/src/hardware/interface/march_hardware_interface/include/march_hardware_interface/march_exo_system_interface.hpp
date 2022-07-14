//
// Created by george on 13-6-22.
//

#ifndef MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
#define MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <rclcpp/logger.hpp>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include <march_hardware/joint.h>
#include <march_hardware/march_robot.h>
#include "rclcpp/macros.hpp"
#include "march_hardware_interface/visibility_control.h"

namespace march_hardware_interface
{
    /// Contains all the needed information for the Hardware Interface for a Joint.
    struct JointInfo
    {
        std::string name;
        march::Joint& joint;
        double position;
        double velocity;
        double effort_command;
        double max_effort;
        double max_velocity;
    };

class MarchExoSystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarchExoSystemInterface);

    MARCH_HARDWARE_INTERFACE_PUBLIC MarchExoSystemInterface();

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

    std::unique_ptr<march::MarchRobot> load_march_hardware(const hardware_interface::HardwareInfo& info) const;
    bool has_correct_actuation_mode(march::Joint& joint) const;

    const std::shared_ptr<rclcpp::Logger> logger_;
    std::unique_ptr<march::MarchRobot> march_robot_;
    std::vector<JointInfo> joints_info_;
    // Store the command for the simulated robot
    std::vector<double> hw_effort_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    std::chrono::steady_clock::time_point last_read_time_;
};

}  // namespace march_hardware_interface

#endif // MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
