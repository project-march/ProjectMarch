#include "state_estimator/footstep_estimator.hpp"

FootstepEstimator::FootstepEstimator()
{
    foot_left.position.header.frame_id = "left_ankle";
    foot_right.position.header.frame_id = "right_ankle";
}

geometry_msgs::msg::PointStamped FootstepEstimator::get_foot_position(const char* prefix)
{
    switch (*prefix) {
        case * "l":
            return foot_left.position;
            break;

        case * "r":
            return foot_right.position;
            break;

        default:
            return geometry_msgs::msg::PointStamped();
    }
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

void FootstepEstimator::update_feet(const std::vector<PressureSensor> sensors)
{
    foot_left.set_on_ground(sensors, "l");
    foot_right.set_on_ground(sensors, "r");
}