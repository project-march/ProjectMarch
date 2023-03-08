// standard
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

#ifndef ZMP_MPC_NODE
#define ZMP_MPC_NODE
using namespace std::chrono_literals;
using std::placeholders::_1;

class SolverNode : public rclcpp::Node {
public:
    SolverNode();

private:
    ZmpSolver m_zmp_solver;

    void gait_callback(trajectory_msgs::msg::JointTrajectory::SharedPtr);

    void robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr);

    void publish_control_msg();
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_trajectory_publisher;
    rclcpp::Subscription<march_shared_msgs::msg::RobotState>::SharedPtr m_robot_state_subscriber;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_gait_subscriber;
};

#endif