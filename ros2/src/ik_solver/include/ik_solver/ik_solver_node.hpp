#ifndef IK_SOLVER_NODE_H
#define IK_SOLVER_NODE_H

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/ik_solver_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs//msg/joint_trajectory.hpp"
#include "trajectory_msgs//msg/joint_trajectory_point.hpp"
#include <chrono>
#include <cstdio>

class IkSolverNode : public rclcpp::Node {
public:
    IkSolverNode();
    void set_foot_placement(geometry_msgs::msg::PoseArray::SharedPtr);

private:
    IkSolver m_ik_solver;
    void com_trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr);
    void swing_trajectory_subscriber_callback(march_shared_msgs::msg::IkSolverCommand::SharedPtr);
    void joint_state_subscriber_callback(sensor_msgs::msg::JointState::SharedPtr);
    void foot_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    void reset_subscriber_callback(std_msgs::msg::Int32::SharedPtr);
    void stance_foot_callback(std_msgs::msg::Int32::SharedPtr);
    void publish_joint_states(std::vector<double>);

    void timer_callback();

    geometry_msgs::msg::PoseArray::SharedPtr m_latest_foot_positions;
    rclcpp::Subscription<march_shared_msgs::msg::IkSolverCommand>::SharedPtr m_com_trajectory_subscriber;
    rclcpp::Subscription<march_shared_msgs::msg::IkSolverCommand>::SharedPtr m_swing_trajectory_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_reset_subscriber;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_foot_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_stance_foot_subscriber;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_publisher;

    rclcpp::TimerBase::SharedPtr m_solving_timer;

    march_shared_msgs::msg::IkSolverCommand::SharedPtr m_com_trajectory_container;
    march_shared_msgs::msg::IkSolverCommand::SharedPtr m_swing_trajectory_container;

    state m_desired_state;
    bool m_right_foot_on_ground;
    int m_swing_trajectory_index;
    int m_com_trajectory_index;
    int m_stance_foot;
    int m_timestep;

    std::vector<std::string> m_joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point_prev_saved;
};

#endif