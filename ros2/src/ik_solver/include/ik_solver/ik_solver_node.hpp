#ifndef IK_SOLVER_NODE_H
#define IK_SOLVER_NODE_H

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cstdio>

class IkSolverNode : public rclcpp::Node {
public:
    IkSolverNode();
    void set_foot_placement(geometry_msgs::msg::PoseArray::SharedPtr);

private:
    IkSolver m_ik_solver;
    void trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr);
    void joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr);
    void foot_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr);

    void timer_callback();

    geometry_msgs::msg::PoseArray::SharedPtr m_latest_foot_positions;
    rclcpp::Subscription<march_shared_msgs::msg::IkSolverCommand>::SharedPtr m_trajectory_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_foot_subscriber;

    rclcpp::TimerBase::SharedPtr m_solving_timer;

    march_shared_msgs::msg::IkSolverCommand::SharedPtr m_trajectory_container;

    state m_desired_state;
    bool m_right_foot_on_ground;
    int m_trajectory_index;
};

#endif