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

/**
 * Set the feet positions.
 * @param right_foot_vec x, y, z coordinates of the right foot.
 * @param left_foot_vec x, y, z coordinates of the left foot.
 */
void FootstepEstimator::set_footstep_positions(
    geometry_msgs::msg::Vector3 right_foot_vec, geometry_msgs::msg::Vector3 left_foot_vec)
{
    foot_right.position.point.x = right_foot_vec.x;
    foot_right.position.point.y = right_foot_vec.y;
    foot_right.position.point.z = right_foot_vec.z;

    foot_left.position.point.x = left_foot_vec.x;
    foot_left.position.point.y = left_foot_vec.y;
    foot_left.position.point.z = left_foot_vec.z;
}

/**
 * Get the position of hte specified foot
 * @param prefix The prefix of the foot, for left the prefix is: "l", for right it is: "r"
 * @return the position of the requested foot.
 */
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

/**
 * Set the size of the foot.
 * @param width The width of the foot.
 * @param height the height of the foot.
 * @param prefix The prefix of the foot, for left the prefix is: "l", for right it is: "r"
 */
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

/**
 * Check if the specified foot is on the ground or not
 * @param prefix The prefix of the foot, for left the prefix is: "l", for right it is: "r"
 * @return True if foot is on the ground, and false otherwise
 */
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

/**
 * Get the requested foot
 * @param prefix The prefix of the foot, for left the prefix is: "l", for right it is: "r"
 * @return The requested foot of it exists, error otherwise
 */
Foot* FootstepEstimator::get_foot(const char* prefix)
{
    switch (*prefix) {
        case * "l":
            return &foot_left;

        case * "r":
            return &foot_right;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("feet_estimator"), "get foot returns 0.");
            return 0;
    }
}

/**
 * Update the current state of the feet based on the pressure soles.
 * @param sensors The pressure sensors with new data.
 */
void FootstepEstimator::update_feet(const std::vector<PressureSensor*>* sensors)
{
    foot_right.set_on_ground(sensors, "r");
    foot_left.set_on_ground(sensors, "l");
}

/**
 * Set the thresholds to determine if the foot is on the ground.
 * @param threshold The threshold of the foot.
 */
void FootstepEstimator::set_threshold(double threshold)
{
    foot_left.threshold = threshold;
    foot_right.threshold = threshold;
}