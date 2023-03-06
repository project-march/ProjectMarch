// standard
#include "zmp_mpc_solver/c_generated_code/main_pendulum_ode.cpp"
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#ifndef ZMP_MPC_H
#define ZMP_MPC_H
using namespace std::chrono_literals;
using std::placeholders::_1;

class SolverNode : public rclcpp::Node {
public:
    SolverNode();
    double x_current[NX];
    double u_current[NU * PENDULUM_ODE_N];
    double m_time_horizon;

private:
    int solve_step(double[], double[]);

    void gait_callback(trajectory_msgs::msg::JointTrajectory::SharedPtr);

    void robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr);

    void publish_control_msg();
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    rclcpp::Subscription<march_shared_msgs::msg::RobotState>::SharedPtr robot_state_subscriber;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr gait_subscriber;
};

#endif