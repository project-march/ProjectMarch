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
#include "march_shared_msgs/msg/joint_gains.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visibility_control.h"
#include <march_hardware/joint.h>
#include <march_hardware/march_robot.h>
#include <march_hardware/motor_controller/odrive/odrive_state.h>
#include <rclcpp/clock.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#define TORQUEDEBUG

using std::placeholders::_1;
using std::placeholders::_2;

namespace march_system_interface {
struct JointLimit {
    int outside_soft_limits_warning_interval_ms; 
    std::chrono::time_point<std::chrono::steady_clock> last_time_within_soft_limits; 
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

    // Values for the fuzzy control on the ODrive
    double position_weight;
    double torque_weight;

    JointLimit limit;
};

// TODO: create a ROS controller that can be used to set the gains
class GainsNode : public rclcpp::Node {
public:
    explicit GainsNode()
        : Node("gains_node")
    {
        m_joint_gains_subscriber = this->create_subscription<march_shared_msgs::msg::JointGains>(
            "joint_gains", 10, std::bind(&GainsNode::jointGainsCallback, this, _1));
    }

    void jointGainsCallback(const march_shared_msgs::msg::JointGains::SharedPtr msg) {
    setJointGains(msg->joint_name, {msg->proportional_gain, msg->integral_gain, msg->derivative_gain});
    }

    void setJointGains(const std::string& joint_name, const std::array<double, 3>& new_position_gains) {
        if (!updateJointGains(joint_name, new_position_gains)) {
            RCLCPP_WARN_ONCE(get_logger(), "Joint '%s' not found!", joint_name.c_str());
        }
    }

    bool updateJointGains(const std::string& joint_name, const std::array<double, 3>& new_position_gains) {
        for (march_system_interface::JointInfo& jointInfo : *joints_info_) {
            if (jointInfo.name == joint_name) {
                jointInfo.joint.setPositionPIDValues(new_position_gains);
                // Commented for now, since the gains are not configured correctly yet
                // jointInfo.joint.sendPID();
                return true;
            }
        }
        return false;
    }

    std::vector<JointInfo>* joints_info_;
private:
    rclcpp::Subscription<march_shared_msgs::msg::JointGains>::SharedPtr m_joint_gains_subscriber;
};

class WeightNode : public rclcpp::Node {
public:
    explicit WeightNode()
        : Node("weight_node")
    {
        m_weight_subscription = this->create_subscription<march_shared_msgs::msg::WeightStamped>(
            "fuzzy_weight", 10, std::bind(&WeightNode::weight_callback, this, _1));

        m_measured_torque_publisher
            = this->create_publisher<control_msgs::msg::JointTrajectoryControllerState>("/measured_torque", 10);

        m_measure_torque_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "/march/measure_torque", 10, std::bind(&WeightNode::average_torque_callback, this, _1));

        RCLCPP_INFO(rclcpp::get_logger("weight_node"), "creating the weight node!");
    }

    /**
     * Processes the weights sent from the fuzzy generator
     *
     * @param msg Message that contains the weights for both torque and position
     * @return
     */
    void weight_callback(march_shared_msgs::msg::WeightStamped::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
            "Weights are in from fuzzy node: joint : " << msg->joint_name << " position " << msg->position_weight
                                                       << ", torque " << msg->torque_weight);

        setJointWeight(msg->joint_name, msg->position_weight, msg->torque_weight);
    }

    /**
     * This is a temporary method: it is used for logging measured torque
     *
     * @return
     */
    void publish_measured_torque()
    {
        control_msgs::msg::JointTrajectoryControllerState torque_points;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (march_system_interface::JointInfo& jointInfo : *joints_info_) {
            // RCLCPP_INFO(this->get_logger(), "Publishing the measured torque: %f", jointInfo.torque);
            torque_points.joint_names.push_back(jointInfo.name);
            point.effort.push_back(jointInfo.torque);
        }

        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 8 * 1e6;
        // torque_points.points.push_back(point);
        torque_points.actual = point;
        torque_points.header.stamp = this->get_clock()->now();

        m_measured_torque_publisher->publish(torque_points);
    }

    /**
     * Applies torque and position weights to all JointInfo objects in the hardware interface
     *
     * @param leg Either "l" or "r" to indicate left or right leg
     * @param position_weight A float between 0 and 1 to apply to joints
     * @param torque_weight A float between 0 and 1 to apply to joints
     * @return
     */
    void setJointWeight(std::string joint_name, float position_weight, float torque_weight)
    {

        // RCLCPP_INFO_STREAM(this->get_logger(), "Setting weights of " << joint_name);

        bool found_joint = false;
        for (march_system_interface::JointInfo& jointInfo : *joints_info_) {
            // RCLCPP_INFO_STREAM(this->get_logger(), "joint name is " << jointInfo.name);
            // if not passing a specific joint, we set the value for all joints
            if (joint_name == "" || jointInfo.name == joint_name) {
                if (jointInfo.torque_weight > std::numeric_limits<float>::epsilon()
                    && (!jointInfo.target_torque || std::isnan(jointInfo.target_torque))) {
                    RCLCPP_FATAL_STREAM(this->get_logger(),
                        "No torque setpoint found for " << joint_name << ". No torque weight will be applied.");
                    return;
                };
                jointInfo.torque_weight = torque_weight;
                jointInfo.position_weight = position_weight;
                found_joint = true;
                break;
            }
        }
        if (!found_joint) {
            RCLCPP_INFO_STREAM(this->get_logger(), "we have not found joint " << joint_name);
        }
    }

    void average_torque_callback(std_msgs::msg::Int32::SharedPtr msg)
    {

        RCLCPP_INFO_STREAM(this->get_logger(), "test log");

        std::map<std::string, std::vector<float>> measured_torques;
        for (auto j : *joints_info_) {
            measured_torques[j.name] = std::vector<float>();
        }
        auto now = std::chrono::steady_clock::now;
        auto work_duration = std::chrono::seconds { msg->data };
        auto start = now();
        // RCLCPP_INFO_STREAM(this->get_logger(), "start: " <<  std::chrono::system_clock::to_time_t(start) << "
        // duration: " << work_duration);
        while ((now() - start) < work_duration) {
            for (auto j : *joints_info_) {
                measured_torques[j.name].push_back(j.torque);
            }
        }

        for (march_system_interface::JointInfo& jointInfo : *joints_info_) {
            std::vector<float> total = measured_torques[jointInfo.name];
            float avg_torque = std::accumulate(total.begin(), total.end(), 0.0) / total.size();
            RCLCPP_INFO_STREAM(this->get_logger(),
                "joint " << jointInfo.name << " has average torque " << avg_torque << " measured over " << total.size()
                         << " values");
            // FIXME: BEUNFIX
            // if (jointInfo.name.compare("left_ankle") == 0 || jointInfo.name.compare("right_ankle") == 0) {
            //     RCLCPP_INFO_STREAM(this->get_logger(), "putting the values into fuzzy!");
            //     jointInfo.target_torque = avg_torque;
            //     jointInfo.torque_weight = 0.4;
            //     jointInfo.position_weight = 0.6;
            // }
            jointInfo.target_torque = avg_torque;
            // Either this or target_torque = jointInfo.joint.torque_sensor.getAverageTorque(); in the cpp if we want to
            // hardcode it
        }
    }

    std::vector<JointInfo>* joints_info_;
    std::optional<float> delta;

private:
    rclcpp::Subscription<march_shared_msgs::msg::WeightStamped>::SharedPtr m_weight_subscription;
    rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr m_measured_torque_publisher;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_measure_torque_subscription;
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

    std::shared_ptr<WeightNode> weight_node;
    std::shared_ptr<GainsNode> gains_node;

private:
    rclcpp::executors::SingleThreadedExecutor executor_; // Executor needed to subscriber
    bool is_joint_in_valid_state(JointInfo& jointInfo);
    bool is_joint_outside_limits(JointInfo& jointInfo);
    JointInfo build_joint_info(const hardware_interface::ComponentInfo& joint);

    std::unique_ptr<march::MarchRobot> load_march_hardware(const hardware_interface::HardwareInfo& info) const;
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
