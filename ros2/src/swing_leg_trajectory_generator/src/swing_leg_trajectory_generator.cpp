//
// Created by march on 8-3-23.
//

#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
#include <math.h>

using PointStamped = geometry_msgs::msg::PointStamped;

SwingLegTrajectoryGenerator::SwingLegTrajectoryGenerator()
{
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
    return m_curve;
}
void SwingLegTrajectoryGenerator::calculateCurve(){
    std::vector<PointStamped> trajectory;
    for (double t = 0.01; t <= 1; t += 0.01) {
        trajectory.push_back(getPoint(t));
    }
    m_curve.trajectory = trajectory;

}
