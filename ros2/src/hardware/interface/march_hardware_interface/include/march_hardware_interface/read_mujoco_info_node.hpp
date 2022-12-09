//
// Created by march on 7-12-22.
//

#ifndef MARCH_READ_MUJOCO_INFO_NODE_HPP
#define MARCH_READ_MUJOCO_INFO_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


struct MjcStateInfo {
    std::string name;
    double mjc_position;
    double mjc_velocity;
    double mjc_effort;
};

class ReadMujocoInfo : public rclcpp::Node
{
    public:
        ReadMujocoInfo();
        std::vector<MjcStateInfo> *mjc_state_info_;

    private:
        void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

#endif // MARCH_READ_MUJOCO_INFO_NODE_HPP
