#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H

#include <march_shared_msgs/msg/current_gait.hpp>
#include <rclcpp/rclcpp.hpp>
#include <march_gazebo_plugins/walk_controller.h>

class ComControllerNode : public rclcpp::Node {
public:
    ComControllerNode(std::shared_ptr<gazebo::ObstacleController> controller);

private:
    void topic_callback(
        const march_shared_msgs::msg::CurrentGait::SharedPtr msg) const;
    rclcpp::Subscription<march_shared_msgs::msg::CurrentGait>::SharedPtr
        subscription_;
    std::shared_ptr<gazebo::ObstacleController> controller_;
};

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H