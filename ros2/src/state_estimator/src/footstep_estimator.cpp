#include "state_estimator/footstep_estimator.hpp"

FootstepEstimator::FootstepEstimator()
{
    foot_left.position.header.frame_id = "left_ankle";
    foot_left.position.point.x = 0.0;
    foot_left.position.point.y = 0.0;
    foot_left.position.point.z = 0.0;
    foot_right.position.header.frame_id = "right_ankle";
    foot_right.position.point.x = 0.0;
    foot_right.position.point.y = 0.0;
    foot_right.position.point.z = 0.0;
}

void FootstepEstimator::set_footstep_positions(
    geometry_msgs::msg::Vector3 right_foot_vec, geometry_msgs::msg::Vector3 left_foot_vec)
{
    foot_right.position.point.x = right_foot_vec.x; // at the start, the CoM is about 0.11 meters in the positive direction, because the 0 is from the right ankle 
    foot_right.position.point.y = right_foot_vec.y;
    foot_right.position.point.z = right_foot_vec.z;

    foot_left.position.point.x = left_foot_vec.x; // at the start, the CoM is about 0.11 meters in the positive direction, because the 0 is from the right ankle 
    foot_left.position.point.y = left_foot_vec.y;
    foot_left.position.point.z = left_foot_vec.z;
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
            RCLCPP_ERROR(rclcpp::get_logger("feet_estimator"), "get foot returns 0");
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
            RCLCPP_ERROR(rclcpp::get_logger("feet_estimator"), "get foot returns 0.");
            return 0;
    }
}

void FootstepEstimator::update_feet(const std::vector<PressureSensor*>* sensors)
{
    foot_right.set_on_ground(sensors, "r");
    foot_left.set_on_ground(sensors, "l");
}

void FootstepEstimator::set_threshold(double threshold)
{
    m_threshold = threshold;
    foot_left.threshold = threshold;
    foot_right.threshold = threshold;
}
