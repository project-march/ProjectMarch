//
// Created by Marco Bak M8 on 8-3-23.
//

#ifndef BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#define BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <string>

class SwingLegTrajectoryGeneratorNode : public rclcpp::Node {
public:
    SwingLegTrajectoryGeneratorNode();

private:


};
#endif //BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
