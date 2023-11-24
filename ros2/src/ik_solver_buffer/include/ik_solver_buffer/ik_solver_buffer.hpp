#ifndef IK_SOLVER_BUFFER_NODE_H
#define IK_SOLVER_BUFFER_NODE_H

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class BufferNode : public rclcpp::Node {
public:
    BufferNode();
    void set_com_trajectory(geometry_msgs::msg::PoseArray::SharedPtr);
    void set_swing_trajectory(geometry_msgs::msg::PoseArray::SharedPtr);

    bool check_if_ready();
    void publish_com_trajectory();
    void publish_swing_trajectory();

private:
    void com_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    void swing_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    void swing_command_subscriber_callback(std_msgs::msg::Int32::SharedPtr);
    void set_velocity(std::vector<geometry_msgs::msg::Point>&, std::vector<geometry_msgs::msg::Point>&, int);
    geometry_msgs::msg::PoseArray::SharedPtr m_latest_com_trajectory;
    geometry_msgs::msg::PoseArray::SharedPtr m_latest_swing_trajectory;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_swing_leg_command_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_com_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_swing_subscriber;

    rclcpp::Publisher<march_shared_msgs::msg::IkSolverCommand>::SharedPtr m_com_trajectory_publisher;
    rclcpp::Publisher<march_shared_msgs::msg::IkSolverCommand>::SharedPtr m_swing_trajectory_publisher;

    int m_timestep;
};

#endif