#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H

#include <iostream>
#include <march_gazebo_plugins/obstacle_controller.h>
#include <march_shared_msgs/msg/current_gait.hpp>
#include <rclcpp/rclcpp.hpp>

class ComControllerNode : public rclcpp::Node {
public:
    ComControllerNode()
        : Node("com_controller_node")
    {
        subscription_
            = this->create_subscription<march_shared_msgs::msg::CurrentGait>(
                "topic", 10,
                std::bind(&ComControllerNode::topic_callback, this,
                    std::placeholders::_1));
        RCLCPP_ERROR(this->get_logger(), "STARTED");
        std::cout << "STARTED" << std::endl;
    }

private:
    void topic_callback(
        const march_shared_msgs::msg::CurrentGait::SharedPtr msg) const
    {
        RCLCPP_ERROR(this->get_logger(), "RECEIVED");
    }
    rclcpp::Subscription<march_shared_msgs::msg::CurrentGait>::SharedPtr
        subscription_;
};

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H