#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <map>
#include <march_gazebo_plugins/obstacle_controller.h>
#include <march_shared_msgs/msg/current_gait.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class ComControllerNode : public rclcpp::Node {
public:
    ComControllerNode(std::shared_ptr<gazebo::ObstacleController> controller, std::map<std::string, int>& pd_values);
    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter>& parameters);
    bool balance_mode();

private:
    rclcpp::Subscription<march_shared_msgs::msg::CurrentGait>::SharedPtr subscription_;
    std::shared_ptr<gazebo::ObstacleController> controller_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    std::map<std::string, int>& pd_values_;
    bool balance_;
};

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_NODE_H