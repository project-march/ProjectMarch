//
// Created by march on 8-3-23.
//

#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
#include <math.h>

using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;

SwingLegTrajectoryGenerator::SwingLegTrajectoryGenerator()
{
    // TODO: Check if these points are correct for the ik-solver, or if they have to be altered for a realistic step.
    m_curve = BezierCurve();
    auto start_point = Point();
    start_point.x = 0;
    start_point.y = 0;
    start_point.z = 0;

    auto left_point = Point();
    left_point.x = 25;
    left_point.y = 50;
    left_point.z = 0;

    auto right_point = Point();
    right_point.x = 75;
    right_point.y = 50;
    right_point.z = 0;

    auto end_point = Point();
    end_point.x = 100;
    end_point.y = 0;
    end_point.z = 0;

    m_curve.points.push_back(start_point);
    m_curve.points.push_back(left_point);
    m_curve.points.push_back(right_point);
    m_curve.points.push_back(end_point);

    m_curve.point_amount = 75; // Based on the shooting nodes in the  ZMP_MPC, that fit in one step.
    m_step_length = 100.0;

    // Generate the trajectory for the first time
    generate_trajectory();
}

void SwingLegTrajectoryGenerator::generate_trajectory()
{
    geometry_msgs::msg::PoseArray trajectory;
    for (int i = 0; i < m_curve.point_amount; i++) {
        geometry_msgs::msg::Pose pose;
        pose.position = get_point(m_curve.points, i);
        trajectory.poses.push_back(pose);
    }
    m_curve.trajectory = trajectory;
}

Point SwingLegTrajectoryGenerator::get_point(std::vector<Point> points, double t)
{
    Point point;
    if (points.size() == 1) {
        return points.at(0);
    } else {
        auto point1 = get_point({ points.begin(), points.end() - 1 }, t);
        auto point2 = get_point({ points.begin() + 1, points.end() }, t);
        double nt = 1. - t;
        point.x = nt * point1.x + t * point2.x;
        point.y = nt * point1.y + t * point2.y;
        point.z = nt * point1.z + t * point2.z;
        return point;
    }
}

void SwingLegTrajectoryGenerator::update_points(std::vector<Point> points, double step_length)
{
    double scalar = step_length / points.back().x;
    for (auto& p : points) {
        p.x *= scalar;
        p.y *= scalar;
        p.z *= scalar;
    }
    m_curve.points = points;
    m_step_length = step_length;
    generate_trajectory();
}

BezierCurve SwingLegTrajectoryGenerator::get_curve()
{
    return m_curve;
}

double SwingLegTrajectoryGenerator::get_step_length()
{
    return m_step_length;
}

void SwingLegTrajectoryGenerator::set_points(std::vector<Point> points)
{
    update_points(points, m_step_length);
}

void SwingLegTrajectoryGenerator::set_step_length(double step_length)
{
    update_points(m_curve.points, step_length);
}
