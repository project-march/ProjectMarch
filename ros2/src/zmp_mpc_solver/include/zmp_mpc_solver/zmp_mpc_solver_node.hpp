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
//#include "march_shared_msgs/msg/robot_state.hpp"
//#include "march_shared_msgs/msg/point_stamped_list.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
// #include "march_shared_msgs/msg/pressure_soles_data.hpp"

#ifndef ZMP_MPC_NODE
#define ZMP_MPC_NODE

class SolverNode : public rclcpp::Node {
public:
    SolverNode();

private:
    ZmpSolver m_zmp_solver;

    void com_callback(geometry_msgs::msg::PointStamped::SharedPtr);
    void zmp_callback(geometry_msgs::msg::PointStamped::SharedPtr);
    void feet_callback(geometry_msgs::msg::PoseArray::SharedPtr);
    //    void robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr);

    void publish_control_msg();

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_trajectory_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_final_feet_publisher;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_com_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_feet_pos_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_zmp_subscriber;

    //    RCLCPP_INFO(this->get_logger(), "Booted up ZMP solver node");
};

#endif