#ifndef TORQUE_CONVERTER_NODE_H
#define TORQUE_CONVERTER_NODE_H

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

class TorqueConverter : public rclcpp::Node {
public:
    TorqueConverter();

private:
    void trajectory_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_trajectory_subscriber;

};

#endif