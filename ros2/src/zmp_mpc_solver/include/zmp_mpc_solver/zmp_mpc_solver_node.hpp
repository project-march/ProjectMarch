// standard
#include <chrono>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

//#include "march_shared_msgs/msg/robot_state.hpp"
//#include "march_shared_msgs/msg/point_stamped_list.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#ifndef ZMP_MPC_NODE
#define ZMP_MPC_NODE

class SolverNode : public rclcpp::Node {
public:
    SolverNode();

private:
    ZmpSolver m_zmp_solver;

    void com_callback(march_shared_msgs::msg::CenterOfMass::SharedPtr);
    void zmp_callback(geometry_msgs::msg::PointStamped::SharedPtr);
    void feet_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    void desired_pos_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    void stance_foot_callback(std_msgs::msg::Int32::SharedPtr);
    void timer_callback();
    void visualize_trajectory();
    void right_foot_ground_callback(std_msgs::msg::Bool::SharedPtr msg);
    void left_foot_ground_callback(std_msgs::msg::Bool::SharedPtr msg);

    double m_desired_previous_foot_x;
    double m_desired_previous_foot_y;

    geometry_msgs::msg::PoseArray::SharedPtr m_desired_footsteps;
    geometry_msgs::msg::PoseArray m_prev_des_footsteps;
    geometry_msgs::msg::PoseArray m_prev_foot_msg;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_trajectory_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_final_feet_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_com_trajectory_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_com_visualizer_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_zmp_visualizer_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m__footstep_visualizer_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_current_shooting_node_publisher;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_right_foot_on_ground_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_left_foot_on_ground_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_stance_foot_subscriber;
    rclcpp::Subscription<march_shared_msgs::msg::CenterOfMass>::SharedPtr m_com_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_feet_pos_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_desired_steps_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_zmp_subscriber;

    rclcpp::TimerBase::SharedPtr m_solving_timer;
};

#endif