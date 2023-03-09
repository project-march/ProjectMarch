//
// Created by march on 8-3-23.
//

#ifndef BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP
#define BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
using PointStamped = geometry_msgs::msg::PointStamped;

struct BezierCurve{
    PointStamped start_point;
    PointStamped end_point;

    // Define parameters
};

class SwingLegTrajectoryGenerator {
public:
    SwingLegTrajectoryGenerator();
};
#endif //BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP
