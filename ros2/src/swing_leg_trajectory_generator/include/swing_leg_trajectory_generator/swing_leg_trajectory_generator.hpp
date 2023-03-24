//
// Created by march on 8-3-23.
//

#ifndef BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP
#define BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
using PointStamped = geometry_msgs::msg::PointStamped;

struct BezierCurve {
    std::vector<PointStamped> points;

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
    PointStamped getPoint(std::vector<PointStamped> points, double t);
    BezierCurve getCurve();
    void setPoints(std::vector<PointStamped> points);

private:
    BezierCurve m_curve;
};
#endif // BUILD_SWING_LEG_TRAJECTORY_GENERATOR_HPP
