//
// Created by george on 13-6-22.
//

#pragma once

#ifndef MARCH_SYSTEM_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
#define MARCH_SYSTEM_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "march_shared_msgs/msg/weight_stamped.hpp"
#include "march_shared_msgs/msg/pid_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visibility_control.h"
#include <march_hardware/joint.h>
#include <march_hardware/march_robot.h>
#include <march_hardware/motor_controller/odrive/odrive_state.h>
#include <fuzzy_weights_controller/fuzzy_weights_controller.hpp>
#include <rclcpp/clock.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace march_system_interface {
struct JointLimit {
    int soft_limit_warning_throttle_msec;
    std::chrono::time_point<std::chrono::steady_clock> last_time_not_in_soft_error_limit;
    std::chrono::milliseconds msec_until_error_when_in_error_soft_limits;
    int soft_error_limit_warning_throttle_msec;
    double max_effort_differance; // Outdated?
    bool stop_when_outside_hard_limits;
};
/// Contains all the needed information for the Hardware Interface for a Joint.
struct JointInfo {
    const std::string name;
    march::Joint& joint;
    march::ODriveState motor_controller_data;
    double position;
    double target_position;
    double velocity;
    double torque;
    double target_torque;

    // TODO: outdated parameters that should be completely removed in the cleanup.
    double effort_actual;
    double effort_command;
    double effort_command_converted;

    // Values for the fuzzy control on the ODrive
    double position_weight;
    double torque_weight;

    JointLimit limit;
};

class GainsNode : public rclcpp::Node {
public:
    explicit GainsNode()
        : Node("gains_node")
    {
        m_pid_values_subscription = this->create_subscription<march_shared_msgs::msg::PidValues>(
            "pid_values", 10, std::bind(&GainsNode::pid_values_callback, this, _1));

        RCLCPP_INFO(rclcpp::get_logger("gains_node"), "Creating the gains node!");
    }

    /**
     * Callback function for the pid_values topic.
     * @param msg The received message.
     */
    void pid_values_callback(const march_shared_msgs::msg::PidValues::SharedPtr msg)
    {      
        setPidValues(msg->joint_name, std::array<double, 3>{msg->proportional_gain, msg->integral_gain, msg->derivative_gain});
    }

    /**
     * Sets the PID values for a joint.
     * @param joint_name The name of the joint.
     * @param new_position_gains The new PID values.
     */
    void setPidValues(std::string joint_name, const std::array<double, 3>& new_position_gains)
    {
        bool jointFound = false;
        for (march_system_interface::JointInfo& jointInfo : *joints_info_) {
            if (jointInfo.name == joint_name) {
                jointInfo.joint.setPositionPIDValues(new_position_gains);
                jointFound = true;

                // Call sendPID() after new PID values are set
                // jointInfo.joint.sendPID();
            }
        }

        if (!jointFound) {
            RCLCPP_WARN_ONCE(get_logger(), "Joint '%s' not found!", joint_name.c_str());
        }
    }

    std::vector<JointInfo>* joints_info_;
private:
    rclcpp::Subscription<march_shared_msgs::msg::PidValues>::SharedPtr m_pid_values_subscription;
};

class MarchExoSystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarchExoSystemInterface);

    MARCH_SYSTEM_INTERFACE_PUBLIC MarchExoSystemInterface();

    MARCH_SYSTEM_INTERFACE_PUBLIC ~MarchExoSystemInterface() override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    hardware_interface::return_type start() override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    hardware_interface::return_type stop() override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    hardware_interface::return_type read() override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    hardware_interface::return_type write() override;

    MARCH_SYSTEM_INTERFACE_PUBLIC
    std::vector<JointInfo>* getJointsInfo()
    {
        return &joints_info_;
    };

    std::shared_ptr<GainsNode> gains_node;

private:
    rclcpp::executors::SingleThreadedExecutor executor_; // Executor needed to subscriber
    void pdb_read();
    bool is_joint_in_valid_state(JointInfo& jointInfo);
    bool is_joint_outside_limits(JointInfo& jointInfo);
    JointInfo build_joint_info(const hardware_interface::ComponentInfo& joint);

    std::unique_ptr<march::MarchRobot> load_march_hardware(const hardware_interface::HardwareInfo& info) const;
    bool has_correct_actuation_mode(march::Joint& joint) const;
    void make_joints_operational(std::vector<march::Joint*> joints);

    const std::shared_ptr<rclcpp::Logger> logger_;
    std::unique_ptr<march::MarchRobot> march_robot_;
    march::PowerDistributionBoardData pdb_data_;
    std::vector<JointInfo> joints_info_;
    bool joints_ready_for_actuation_ = false;
    rclcpp::Clock clock_;
};

} // namespace march_system_interface

#endif // MARCH_SYSTEM_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
