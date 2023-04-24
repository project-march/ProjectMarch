#ifndef TORQUE_CONVERTER_NODE_H
#define TORQUE_CONVERTER_NODE_H

#include "torque_converter/torque_converter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>

class TorqueConverterNode : public rclcpp::Node {
public:
    TorqueConverterNode();

private:
    TorqueConverter m_torque_converter;
    
    void trajectory_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_trajectory_subscriber;
};

#endif
