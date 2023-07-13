//
// Created by march on 8-3-23.
//

#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
#include <cmath>

using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;

SwingLegTrajectoryGenerator::SwingLegTrajectoryGenerator()
{
    m_curve = BezierCurve();
    m_curve.point_amount = 75; // Based on the shooting nodes in the  ZMP_MPC, that fit in one step.
    m_step_length = 0.2;
}

/**
 * Generate the swing leg trajectory.
 * This recursive algorithm creates the bezier trajectory that should be followed by the swing leg of the exo.
 */
void SwingLegTrajectoryGenerator::generate_trajectory()
{
    geometry_msgs::msg::PoseArray trajectory;
    double step_size = 1.0 / m_curve.point_amount;
    for (int i = 0; i <= m_curve.point_amount; i++) {
        double t = step_size * static_cast<double>(i);
        geometry_msgs::msg::Pose pose;
        auto points = m_curve.points;
        pose.position = get_point(points, t);
        trajectory.poses.push_back(pose);
    }
    m_curve.trajectory = trajectory;
}

/**
 * Get a specific point from the curve for a specified t between 0 and 1.
 * @param points The points with which the bezier curve is calculated
 * @param t The t for which we want to calculate the point on the curve, has range 0.0 to 1.0
 * @return The calculated point.
 */
Point SwingLegTrajectoryGenerator::get_point(std::vector<Point> points, double t)
{
    Point point;
    if (points.size() == 1) {
        return points.at(/*__n=*/0);
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

/**
 * Update the four points on which the bezier curve is based.
 * Because the bezier visualization uses a different scale then the swing leg trajectory generator, here the points are
 * updated with a scalar.
 * @param points The new points which needs to be updated
 * @param step_length The step length for which the points need to be updated
 */
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

/**
 * Return the bezier curve that is generated.
 * @return
 */
BezierCurve SwingLegTrajectoryGenerator::get_curve()
{
    return m_curve;
}

/**
 * Get the step length that is used to base the curve on.
 * @return
 */
double SwingLegTrajectoryGenerator::get_step_length()
{
    return m_step_length;
}

/**
 * Set new points for the bezier curve.
 * These new points have to be scaled with the step length in the update points function.
 * @param points
 */
void SwingLegTrajectoryGenerator::set_points(std::vector<Point> points)
{
    update_points(points, m_step_length);
}

/**
 * Set a new step length to the bezier curve.
 * This step length should also be used to scale the points again with the update points function.
 * @param step_length
 */
void SwingLegTrajectoryGenerator::set_step_length(double step_length)
{
    update_points(m_curve.points, step_length);
}
