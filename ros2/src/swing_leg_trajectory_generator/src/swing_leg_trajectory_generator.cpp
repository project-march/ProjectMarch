//
// Created by march on 8-3-23.
//

#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
#include <math.h>

using PointStamped = geometry_msgs::msg::PointStamped;

SwingLegTrajectoryGenerator::SwingLegTrajectoryGenerator()
{
    // TODO: Check if these points are correct for the ik-solver, or if they have to be altered for a realistic step.
    m_curve = BezierCurve();
    auto start_point = PointStamped();
    start_point.point.x = 0;
    start_point.point.y = 0;
    start_point.point.z = 0;

    auto left_point = PointStamped();
    left_point.point.x = 25;
    left_point.point.y = 50;
    left_point.point.z = 0;

    auto right_point = PointStamped();
    right_point.point.x = 75;
    right_point.point.y = 50;
    right_point.point.z = 0;

    auto end_point = PointStamped();
    end_point.point.x = 100;
    end_point.point.y = 0;
    end_point.point.z = 0;

    m_curve.points.push_back(start_point);
    m_curve.points.push_back(left_point);
    m_curve.points.push_back(right_point);
    m_curve.points.push_back(end_point);
}

PointStamped SwingLegTrajectoryGenerator::getPoint(std::vector<PointStamped> points, double t)
{
    PointStamped point;
    if (points.size() == 1) {
        return points.at(0);
    } else {
        auto point1 = getPoint({ points.begin(), points.end() - 1 }, t);
        auto point2 = getPoint({ points.begin() + 1, points.end() }, t);
        double nt = 1. - t;
        point.point.x = nt * point1.point.x + t * point2.point.x;
        point.point.y = nt * point1.point.y + t * point2.point.y;
        point.point.z = nt * point1.point.z + t * point2.point.z;
        return point;
    }
}

BezierCurve SwingLegTrajectoryGenerator::getCurve()
{
    return m_curve;
}

void SwingLegTrajectoryGenerator::setPoints(std::vector<PointStamped> points)
{
    m_curve.points = points;
}
