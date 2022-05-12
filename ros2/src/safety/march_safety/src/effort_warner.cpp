/**
 * @author George Vegelien - MARCH 7
 * @copyright 2019 Project March.
 */
#include "../include/march_safety/effort_warner.h"
using namespace rclcpp;

/**
 * Constructs an EffortWarner object to keep count that joints do not
 * consistently get high efforts.
 *
 * Changeable values/parameters: 'max_effort_limit', 'max_effort_time_limit' and
 * 'time_between_effort_warnings'.
 *
 * This warner listens on the topic "/march/joint_states".
 *
 * @param node A reference to the node to which this warner attaches itself
 * (from where it gets its param values, logs and listens to topics).
 * @param joint_names A list of joint names that the warner keeps track of.
 */
EffortWarner::EffortWarner(Node& node, std::vector<std::string>& joint_names)
    : logger_(node.get_logger())
    , joint_names_(joint_names)
{
    this->under_effort_timestamps_.resize(joint_names.size(), node.now());
    this->last_warning_timestamp_.resize(joint_names.size(),
        Time(/*seconds=*/0, /*nanoseconds=*/0, RCL_ROS_TIME));
    this->max_time_ = node.get_parameter("max_effort_time_limit").as_double();
    this->time_between_warnings_
        = node.get_parameter("time_between_effort_warnings").as_double();
    this->max_effort_
        = node.get_parameter("max_effort_limit").get_value<float>();
    this->subscription_ = node.create_subscription<JointStateMsg>(
        "/march/joint_states", SystemDefaultsQoS(),
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const JointStateMsg::SharedPtr msg_ptr)
            -> void {
            effortValueCallback(msg_ptr);
        });
}

/**
 * Callback function for when a message is published on '/march/joint_states'.
 *
 * This function compares the effort for every joint in the
 * `this->joint_names_`, if the effort exceeds (inclusive) the
 * `this->max_effort_` for more then `this->max_time_` seconds and it hasn't
 * warned in the last `this->time_between_warnings_` seconds, then it gives a
 * ROS warning.
 *
 * @param msg_ptr A shared pointer to the message from where it reads the time
 * and the effort sent.
 */
void EffortWarner::effortValueCallback(const JointStateMsg::SharedPtr& msg_ptr)
{
    int i = 0;
    Time current_time = Time(msg_ptr->header.stamp);
    for (const std::string& joint_name : joint_names_) {
        double time_diff_sec
            = (current_time - under_effort_timestamps_[i]).seconds();
        if (std::abs(msg_ptr->effort[i]) >= max_effort_) {
            bool msg_recently_published = time_between_warnings_
                > ((current_time - last_warning_timestamp_[i]).seconds());
            if (time_diff_sec >= max_time_ && !msg_recently_published) {
                last_warning_timestamp_[i] = current_time;
                RCLCPP_WARN(logger_,
                    "The %s gets an effort above %f for %f seconds",
                    joint_name.c_str(), max_effort_, time_diff_sec);
            }
        } else {
            under_effort_timestamps_[i] = current_time;
        }
        i++;
    }
}
