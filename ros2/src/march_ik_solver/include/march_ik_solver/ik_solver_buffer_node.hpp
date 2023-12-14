#ifndef IK_SOLVER__IK_SOLVER_BUFFER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_BUFFER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "march_shared_msgs/msg/exo_state.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class IKSolverBufferNode : public rclcpp::Node
{
    public:
        IKSolverBufferNode();

    private:
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
        void exoStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg);
        void ikSolverFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
        void publishIKSolverFootPositions();
        void publishIKSolverStatus();
        void publishIKSolverError(double error);

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;
        rclcpp::Subscription<march_shared_msgs::msg::ExoState>::SharedPtr exo_state_sub_;
        rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr ik_solver_foot_positions_sub_;
        rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr ik_solver_foot_positions_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr ik_solver_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ik_solver_error_pub_;

        bool gait_reset_;
        int8_t gait_type_;
        unsigned int n_joints_;
        double dt_;
        double convergence_threshold_;
        bool early_stopping_;
        long unsigned int early_stopping_threshold_;
        Eigen::VectorXd current_joint_positions_;
        Eigen::VectorXd desired_joint_positions_;
        std::vector<march_shared_msgs::msg::IksFootPositions> ik_solver_foot_positions_buffer_;
        march_shared_msgs::msg::IksFootPositions ik_solver_foot_positions_latest_;
        bool ik_solver_foot_positions_received_;
};

#endif // IK_SOLVER__IK_SOLVER_BUFFER_NODE_HPP_