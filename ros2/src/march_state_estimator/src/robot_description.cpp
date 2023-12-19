#include "march_state_estimator/robot_description.hpp"

#include "rclcpp/rclcpp.hpp"

RobotDescription::RobotDescription()
{
    
}

void RobotDescription::parse(const std::string & urdf_path)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescription::parse");
    urdf_model_.initFile(urdf_path);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescription::parse done");
}