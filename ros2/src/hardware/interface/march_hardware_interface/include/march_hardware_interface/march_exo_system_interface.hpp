//
// Created by george on 13-6-22.
//

#pragma once

#ifndef MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
#define MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_

#include <functional>
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
#include "visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <march_hardware/joint.h>
#include <march_hardware/march_robot.h>
#include <march_hardware/motor_controller/odrive/odrive_state.h>
#include <rclcpp/clock.hpp>
#include "march_shared_msgs/msg/weight_stamped.hpp"
#include <std_msgs/msg/float32.hpp>
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#define TORQUEDEBUG


using std::placeholders::_1;
using std::placeholders::_2;

namespace march_hardware_interface {
struct JointLimit {
    int soft_limit_warning_throttle_msec;
    std::chrono::time_point<std::chrono::steady_clock> last_time_not_in_soft_error_limit;
    std::chrono::milliseconds msec_until_error_when_in_error_soft_limits;
    int soft_error_limit_warning_throttle_msec;
    double max_effort_differance;
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
    double effort_actual;
    double effort_command;
    double effort_command_converted;

    // Values for the fuzzy control on the ODrive
    double position_weight;
    double torque_weight;

    JointLimit limit;
};

    class WeightNode : public rclcpp::Node {
    public:
        explicit WeightNode()
                : Node("weight_node")
        {
            m_weight_subscription = this->create_subscription<march_shared_msgs::msg::WeightStamped>(
                    "fuzzy_weight", 10, std::bind(&WeightNode::weight_callback, this, _1));

            m_direct_torque_subscription = this->create_subscription<std_msgs::msg::Float32>(
                    "/march/direct_torque", 10, std::bind(&WeightNode::direct_torque_callback, this, _1));

            m_measured_torque_publisher = this->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
                    "/measured_torque", 10);

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
            #ifdef TORQUEDEBUG
            RCLCPP_INFO(this->get_logger(), "Weights are in from fuzzy node: position %f, torque %f", msg->position_weight, msg->torque_weight);
            // return;
            #endif
            // RCLCPP_INFO(this->get_logger(), "Ignoring calculated weight: position %f, torque %f", msg->position_weight, msg->torque_weight);
            setJointsWeight(msg->leg, msg->position_weight, msg->torque_weight);
        }

        /**
         * This is a temporary method: it immediately sends out torque using a delta
         *
         * @param msg Message that contains the weights for both torque and position
         * @return
         */
        void direct_torque_callback(std_msgs::msg::Float32::SharedPtr msg)
        {
            delta = msg->data;
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
            for(march_hardware_interface::JointInfo& jointInfo : *joints_info_){
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
        void setJointsWeight(std::string leg, float position_weight, float torque_weight){

            // check the leg choice
            if(leg != "l" and leg != "r"){
                RCLCPP_WARN(this->get_logger(), "Invalid character provided in weight message: %c! Provide either 'l' or 'r'.", leg);
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Setting weights of" << leg);

            for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
                RCLCPP_INFO_STREAM(this->get_logger(), "joint name is " << jointInfo.name);
                if(leg == "l" and jointInfo.name.find("left") != std::string::npos){
                    jointInfo.torque_weight = torque_weight;
                    jointInfo.position_weight = position_weight;
                }
                else if(leg == "r" and jointInfo.name.find("right") != std::string::npos){
                    jointInfo.torque_weight = torque_weight;
                    jointInfo.position_weight = position_weight;
                }
                else{
                    RCLCPP_WARN(this->get_logger(), "Joint %s seems to be part of neither the right nor the left leg... We are likely using the TSU", jointInfo.name);
                    jointInfo.torque_weight = torque_weight;
                    jointInfo.position_weight = position_weight;
                }
            }
        }

        std::vector<JointInfo>* joints_info_;
        std::optional<float> delta;

    private:
        rclcpp::Subscription<march_shared_msgs::msg::WeightStamped>::SharedPtr m_weight_subscription;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_direct_torque_subscription;
        rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr m_measured_torque_publisher;
    };

class MarchExoSystemInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarchExoSystemInterface);

    MARCH_HARDWARE_INTERFACE_PUBLIC MarchExoSystemInterface();

    MARCH_HARDWARE_INTERFACE_PUBLIC ~MarchExoSystemInterface() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type start() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type stop() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write() override;

    MARCH_HARDWARE_INTERFACE_PUBLIC
    std::vector<JointInfo>* getJointsInfo(){
        return &joints_info_ ;
    };

    std::shared_ptr<WeightNode> weight_node;

private:
    rclcpp::executors::SingleThreadedExecutor executor_; // Executor needed to subscriber
    void pdb_read();
    void pressure_sole_read();
    bool is_joint_in_valid_state(JointInfo& jointInfo);
    bool is_joint_in_limit(JointInfo& jointInfo);
    JointInfo build_joint_info(const hardware_interface::ComponentInfo& joint);

    std::unique_ptr<march::MarchRobot> load_march_hardware(const hardware_interface::HardwareInfo& info) const;
    bool has_correct_actuation_mode(march::Joint& joint) const;
    void make_joints_operational(std::vector<march::Joint*> joints);

    const std::shared_ptr<rclcpp::Logger> logger_;
    std::unique_ptr<march::MarchRobot> march_robot_;
    march::PowerDistributionBoardData pdb_data_;
    std::vector<march::PressureSoleData> pressure_soles_data_;
    std::vector<JointInfo> joints_info_;
    bool joints_ready_for_actuation_ = false;
    rclcpp::Clock clock_;
};

} // namespace march_hardware_interface

#endif // MARCH_HARDWARE_INTERFACE__MARCH_EXO_SYSTEM_INTERFACE_HPP_
