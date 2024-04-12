//
// Created by george on 13-6-22.
//

#ifndef MARCH_SYSTEM_INTERFACE__MARCH_MOCK_SYSTEM_INTERFACE_HPP_
#define MARCH_SYSTEM_INTERFACE__MARCH_MOCK_SYSTEM_INTERFACE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
#include "march_system_interface/march_mock_system_interface.hpp"
#include "march_system_interface/visibility_control.h"
#include "rclcpp/macros.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace march_system_interface {

/// Contains all the needed information for the Hardware Interface.
struct HwStateInfo {
    std::string name;
    double hw_position;
    double hw_velocity;
    double hw_effort;
};

class SimCommunication : public rclcpp::Node {
public:
    explicit SimCommunication()
        : Node("simulation_communication")
    {
        m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&SimCommunication::sim_callback, this, _1));
    }

    void sim_callback(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        hw_positions_ = msg->position;
    }

    std::vector<double> get_pos()
    {
        return hw_positions_;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    std::vector<double> hw_positions_;
};

class MarchMockSystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarchMockSystemInterface);

    MARCH_SYSTEM_INTERFACE_PUBLIC
    MarchMockSystemInterface();

    MARCH_SYSTEM_INTERFACE_PUBLIC
    ~MarchMockSystemInterface() override;

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type start() override;

    hardware_interface::return_type stop() override;

    hardware_interface::return_type read() override;

    hardware_interface::return_type write() override;

    std::shared_ptr<SimCommunication> comms;

private:
    rclcpp::executors::SingleThreadedExecutor executor_; // Executor needed to subscriber
    const std::shared_ptr<rclcpp::Logger> logger_;
    static const std::string COMMAND_AND_STATE_TYPE; // = hardware_interface::HW_IF_POSITION
    std::vector<HwStateInfo> hw_state_info_;
    std::vector<double> hw_positions_;

    march::PowerDistributionBoardData pdb_data_;
    std::vector<march::ODriveState> motor_controllers_data_;
};

} // namespace march_system_interface

#endif // MARCH_SYSTEM_INTERFACE__MARCH_MOCK_SYSTEM_INTERFACE_HPP_
