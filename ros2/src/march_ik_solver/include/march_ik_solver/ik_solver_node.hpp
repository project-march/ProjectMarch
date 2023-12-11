#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "march_ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include "rclcpp/rclcpp.hpp"

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode();

private:
    void IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishJointTrajectory();
    void calculateDesiredJointStates();
    void currentJointPositionsCallback(
        const rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture future);
    std::vector<Eigen::VectorXd> calculateError();
    trajectory_msgs::msg::JointTrajectory convertToJointTrajectoryMsg();

    IKSolver ik_solver_;
    Eigen::VectorXd current_joint_positions_;
    Eigen::VectorXd current_joint_velocities_;
    Eigen::VectorXd desired_joint_positions_;
    Eigen::VectorXd desired_joint_velocities_;
    std::vector<Eigen::VectorXd> desired_poses_;
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_prev_;

    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr ik_solver_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr current_joint_positions_client_;
    rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture current_joint_positions_future_;
    march_shared_msgs::srv::GetCurrentJointPositions::Request::SharedPtr current_joint_positions_request_;
};

#endif // IK_SOLVER__IK_SOLVER_NODE_HPP_