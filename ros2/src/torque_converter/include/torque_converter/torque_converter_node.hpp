#ifndef TORQUE_CONVERTER_NODE_H
#define TORQUE_CONVERTER_NODE_H

#include "torque_converter/torque_converter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include <chrono>
#include <cstdio>

class TorqueConverterNode : public rclcpp::Node {
public:
    TorqueConverterNode();

private:
    TorqueConverter m_torque_converter;
    
    //subscriber things
    void joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr);
    void trajectory_subscriber_callback(control_msgs::msg::JointTrajectoryControllerState::SharedPtr);
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_subscriber;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr m_trajectory_subscriber;

    //publisher things
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr m_torque_trajectory_publisher;

};

#endif
