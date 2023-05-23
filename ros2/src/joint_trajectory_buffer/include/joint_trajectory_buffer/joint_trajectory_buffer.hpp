#ifndef JOINT_TRAJECTORY_BUFFER_NODE_HPP
#define JOINT_TRAJECTORY_BUFFER_NODE_HPP

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include <queue>

class TrajectoryBufferNode : public rclcpp::Node {
public:
    TrajectoryBufferNode();

private:
    void torque_trajectory_callback(trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr);
    void position_trajectory_callback(trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr);
    void publish_joint_trajectory();

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr m_torque_trajectory_subscriber;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr m_position_trajectory_subscriber;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_publisher;

    std::queue<trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr> m_torque_buffer;
    std::queue<trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr> m_position_buffer;

    // {position, torque}
    std::tuple<trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr, trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr> current;
};

#endif