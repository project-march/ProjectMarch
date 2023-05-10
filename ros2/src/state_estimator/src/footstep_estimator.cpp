#include "state_estimator/footstep_estimator.hpp"

FootstepEstimator::FootstepEstimator()
{
    foot_left.position.header.frame_id = "left_ankle";
    foot_right.position.header.frame_id = "right_ankle";
}

geometry_msgs::msg::Pose FootstepEstimator::get_foot_position(const char* prefix)
{
    geometry_msgs::msg::Pose foot_pose;

    switch (*prefix) {
        case * "l":
            foot_pose.position = foot_left.position.point;
            break;

        case * "r":
            foot_pose.position = foot_right.position.point;
            break;

        default:
            return geometry_msgs::msg::Pose();
    }

    return foot_pose;
}

void FootstepEstimator::set_foot_size(double width, double height, const char* prefix)
{
    switch (*prefix) {
        case * "l":
            foot_left.width = width;
            foot_left.height = height;
            break;

        case * "r":
            foot_right.width = width;
            foot_right.height = height;
            break;
    }
}

bool FootstepEstimator::get_foot_on_ground(const char* prefix)
{
    switch (*prefix) {
        case * "l":
            return foot_left.on_ground;
            break;

        case * "r":
            return foot_right.on_ground;
            break;
        default:
            return 0;
    }
}

Foot* FootstepEstimator::get_foot(const char* prefix)
{
    switch (*prefix) {
        case * "l":
            return &foot_left;
            break;

        case * "r":
            return &foot_right;
            break;
        default:
            return 0;
    }
}

void FootstepEstimator::update_feet(const std::vector<PressureSensor*> sensors)
{
    foot_left.set_on_ground(sensors, "l");
    foot_right.set_on_ground(sensors, "r");
}

void FootstepEstimator::set_threshold(double threshold)
{
    m_threshold = threshold;
    foot_left.threshold = threshold;
    foot_right.threshold = threshold;
}