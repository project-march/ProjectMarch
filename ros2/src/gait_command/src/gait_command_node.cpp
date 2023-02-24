//
// Created by Marco Bak on 23-2-23.
//
#include "gait_command/gait_command_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

GaitCommandNode::GaitCommandNode()
    : Node("gait_command_node")
{
    m_ipd_subscriber = this->create_subscription<march_shared_msgs::msg::IpdInput>(
        "ipd_command", 10, std::bind(&GaitCommandNode::ipd_callback, this, _1));
    m_gait_type_publisher = this->create_publisher<march_shared_msgs::msg::GaitType>("gait_type", 10);
};

void GaitCommandNode::ipd_callback(march_shared_msgs::msg::IpdInput::SharedPtr msg)
{
    auto gait_msg = march_shared_msgs::msg::GaitType();
    gait_msg.gait_type = msg->input_cmd;
    m_gait_type_publisher->publish(gait_msg);
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitCommandNode>());
    rclcpp::shutdown();
    return 0;
}
