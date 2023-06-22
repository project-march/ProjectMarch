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
            setJointsWeight(msg->leg, msg->position_weight, msg->torque_weight);
        }

        /**
         * This is a temporary method: it immediately sends out torque
         *
         * @param msg Message that contains the weights for both torque and position
         * @return
         */
        void direct_torque_callback(std_msgs::msg::Float32::SharedPtr msg)
        {

            delta = msg->data;
            // #ifdef TORQUEDEBUG
            // RCLCPP_FATAL(this->get_logger(), "Direct torque called with a delta of: %f", delta);
            // // return;
            // #endif
            //
            // for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
            //     if(jointInfo.name == "rotational_joint" || jointInfo.name == "linear_joint"){
            //         // auto p = jointInfo.joint.getMotorController()->getAbsoluteEncoder()->getTotalPositions(); //*jointInfo.joint.getMotorController()->getAbsoluteEncoder()->getRadiansPerIU();
            //         // RCLCPP_INFO_STREAM(this->get_logger(), "Measured position: " << jointInfo.position << "p: " << p);
            // 
            //         // float c = cos(jointInfo.position);
            //         // RCLCPP_INFO(this->get_logger(), "Measured torque: %f \n Delta: %f \n  C: %f \n", jointInfo.torque, delta, c);
            // 
            //         // float torque = jointInfo.torque + (delta * c);
            //
            //         // We are setting the torque immediately
            //         // RCLCPP_INFO(this->get_logger(), "We are setting the torque directly to %f for %s", torque, jointInfo.name);
            //         // if(jointInfo.torque_weight != 1.0 || jointInfo.position_weight != 0.0){
            //         //     RCLCPP_WARN(rclcpp::get_logger("weight_node"), "We seem to be not completely in torque control: pos weight %f, torque weight %f. \n so we will set it now to 0 pos and 1 torque",
            //         //                 jointInfo.position_weight,
            //         //                 jointInfo.torque_weight);
            //                                        
            //         // // comment back in for correct torque control
            //         // jointInfo.position_weight = 0.0f;
            //         // jointInfo.torque_weight = 1.0f;
            //         // }
            //
            //         // jointInfo.target_torque = torque;
            //         // RCLCPP_INFO(this->get_logger(), "Target torque: %f \n ", jointInfo.target_torque);
            //     }
            //     else{
            //         RCLCPP_INFO_STREAM(this->get_logger(), "It is not wise to use the direct torque button in Hennie! " << jointInfo.name);
            //     }
            // }
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
                // return;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Setting weights of" << leg);

            for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
                RCLCPP_INFO_STREAM(this->get_logger(), "joint name is" << jointInfo.name);
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
        float c = 0.0f;

    private:
        rclcpp::Subscription<march_shared_msgs::msg::WeightStamped>::SharedPtr m_weight_subscription;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_direct_torque_subscription;
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
