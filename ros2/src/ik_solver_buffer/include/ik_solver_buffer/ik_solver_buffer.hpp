#ifndef IK_SOLVER_BUFFER_NODE_H
#define IK_SOLVER_BUFFER_NODE_H

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
#include "rclcpp/rclcpp.hpp"

class BufferNode : public rclcpp::Node {
public:
    BufferNode();
    void set_com_trajectory(geometry_msgs::msg::PoseArray::SharedPtr);
    void set_swing_trajectory(geometry_msgs::msg::PoseArray::SharedPtr);

    bool check_if_ready();
    void publish_ik_trajectory();

private:
    void com_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    void swing_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    void set_velocity(std::vector<geometry_msgs::msg::Point>&, std::vector<geometry_msgs::msg::Point>&);
    geometry_msgs::msg::PoseArray::SharedPtr m_latest_com_trajectory;
    geometry_msgs::msg::PoseArray::SharedPtr m_latest_swing_trajectory;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_com_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_swing_subscriber;

    rclcpp::Publisher<march_shared_msgs::msg::IkSolverCommand>::SharedPtr m_buffer_publisher;

    int m_timestep;
};

#endif