#ifndef IK_SOLVER_NODE_H
#define IK_SOLVER_NODE_H

#include "geometry_msgs/msg/point_stamped.hpp"
#include "ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cstdio>

class IkSolverNode : public rclcpp::Node {
public:
    IkSolverNode();
    void set_foot_placement(geometry_msgs::msg::PointStamped::SharedPtr);

private:
    IkSolver m_ik_solver;
    void trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr);
    void joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr);
    void foot_subscriber_callback(geometry_msgs::msg::PointStamped::SharedPtr);

    geometry_msgs::msg::PointStamped::SharedPtr m_latest_placed_foot;
    rclcpp::Subscription<march_shared_msgs::msg::IkSolverCommand>::SharedPtr m_trajectory_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_foot_subscriber;
};

#endif