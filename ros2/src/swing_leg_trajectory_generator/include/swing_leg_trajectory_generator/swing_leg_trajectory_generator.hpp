//
// Created by march on 8-3-23.
//

#ifndef BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP
#define BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;

struct BezierCurve {
    std::vector<Point> points;
    geometry_msgs::msg::PoseArray trajectory;
    int point_amount;

    /**
     * This function is in place for comparing the struct.
     * This is needed for testing purposes.
     * @param a
     * @param b
     * @return
     */
    friend bool operator==(BezierCurve a, BezierCurve b)
    {
        if (a.points.size() != b.points.size()) {
            return false;
        }
        for (size_t i = 0; i < a.points.size(); i++) {
            if (a.points.at(i) != b.points.at(i)) {
                return false;
            }
        }
        return true;
    }
};

class SwingLegTrajectoryGenerator {
public:
    SwingLegTrajectoryGenerator();
    Point get_point(std::vector<Point> points, double t);
    void generate_trajectory();
    BezierCurve get_curve();
    double get_step_length();
    void set_points(std::vector<Point> points);
    void set_step_length(double step_length);
    void update_points(std::vector<Point> points, double step_length);

private:
    BezierCurve m_curve;
    double m_step_length;
};
#endif // BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP
