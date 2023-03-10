//
// Created by march on 8-3-23.
//

#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
#include <math.h>

using PointStamped = geometry_msgs::msg::PointStamped;

SwingLegTrajectoryGenerator::SwingLegTrajectoryGenerator()
{
    m_curve = BezierCurve();
    auto start_point = PointStamped();
    start_point.point.x = 1;
    start_point.point.y = 0;
    start_point.point.z = 0;

    auto left_point = PointStamped();
    left_point.point.x = 25;
    left_point.point.y = 50;
    left_point.point.z = 0;

    auto right_point = PointStamped();
    right_point.point.x = 75;
    right_point.point.y = 75;
    right_point.point.z = 0;

    auto end_point = PointStamped();
    end_point.point.x = 99;
    end_point.point.y = 0;
    end_point.point.z = 0;

    m_curve.points.push_back(start_point);
    m_curve.points.push_back(left_point);
    m_curve.points.push_back(right_point);
    m_curve.points.push_back(end_point);

    m_curve.start_point = start_point;
    m_curve.left_point = left_point;
    m_curve.right_point = right_point;
    m_curve.end_point = end_point;
    calculateCurve();
}

PointStamped SwingLegTrajectoryGenerator::getPoint(double t)
{
    PointStamped point;

    point.point.x = std::pow((1 - t), 3) * m_curve.start_point.point.x
        + 3 * std::pow((1 - t), 2) * t * m_curve.left_point.point.x
        + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * m_curve.right_point.point.x
        + std::pow(t, 3) * m_curve.end_point.point.x;
    point.point.y = std::pow((1 - t), 3) * m_curve.start_point.point.y
        + 3 * std::pow((1 - t), 2) * t * m_curve.left_point.point.y
        + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * m_curve.right_point.point.y
        + std::pow(t, 3) * m_curve.end_point.point.y;
    point.point.z = std::pow((1 - t), 3) * m_curve.start_point.point.z
        + 3 * std::pow((1 - t), 2) * t * m_curve.left_point.point.z
        + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * m_curve.right_point.point.z
        + std::pow(t, 3) * m_curve.end_point.point.z;

    return point;
}

BezierCurve SwingLegTrajectoryGenerator::getBezier()
{
    calculateCurve();
    return m_curve;
}
void SwingLegTrajectoryGenerator::calculateCurve()
{
    std::vector<PointStamped> trajectory;
    for (double t = 0.01; t <= 1; t += 0.01) {
        trajectory.push_back(getPoint(t));
    }
    m_curve.trajectory = trajectory;
}

void SwingLegTrajectoryGenerator::setPoints(std::vector<PointStamped> points)
{
    m_curve.points = points;
}
