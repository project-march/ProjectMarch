#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <future>
#include <thread>
#include <functional>

#include "march_ik_solver/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"

class IKSolverNode : public rclcpp::Node
{
    public:
        IKSolverNode();

    private:
        void timerCallback();
        void IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
        void RobotStateCallback(const march_shared_msgs::msg::RobotState::SharedPtr msg);
        void publishJointState(const Eigen::VectorXd desired_joint_positions, const Eigen::VectorXd desired_joint_velocities);
        void calculateDesiredJointStates();
        void currentJointPositionsCallback(
            const rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture future);
        
        IKSolver ik_solver_;
        Eigen::VectorXd current_joint_positions_;
        Eigen::VectorXd desired_joint_positions_;
        Eigen::VectorXd desired_joint_velocities_;
        std::vector<Eigen::VectorXd> desired_poses_;

        // rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr ik_solver_command_sub_;
        rclcpp::Subscription<march_shared_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
        rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
        rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr current_joint_positions_client_;
        rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture current_joint_positions_future_;
        march_shared_msgs::srv::GetCurrentJointPositions::Request::SharedPtr current_joint_positions_request_;
        
};

#endif  // IK_SOLVER__IK_SOLVER_NODE_HPP_