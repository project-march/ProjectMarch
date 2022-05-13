/**
 * @author George Vegelien - MARCH 7
 * @copyright 2019 Project March.
 */
#ifndef MARCH_REALSENSE_READER_EFFORT_WARNER_H
#define MARCH_REALSENSE_READER_EFFORT_WARNER_H
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <vector>

class EffortWarner {
    using JointStateMsg = sensor_msgs::msg::JointState;

public:
    EffortWarner(rclcpp::Node& node, std::vector<std::string>& joint_names);

private:
    void effortValueCallback(const JointStateMsg::SharedPtr& msg_ptr);

    const rclcpp::Logger logger_;
    const std::vector<std::string>& joint_names_;
    std::vector<rclcpp::Time> under_effort_timestamps_;
    std::vector<rclcpp::Time> last_warning_timestamp_;
    /// The time (in seconds) it may take having an effort > max_effort_ before
    /// sending effort warnings.
    double max_time_;
    /// The time between warnings in seconds.
    double time_between_warnings_;
    /// The effort which it may not exceed for `max_time_` seconds.
    float max_effort_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr subscription_;
};

#endif // MARCH_REALSENSE_READER_EFFORT_WARNER_H
