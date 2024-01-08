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

        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_sub;
        rclcpp::Subscription<march_shared_msgs::msg::ExoState>::SharedPtr m_exo_state_sub;
        rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_ik_solver_foot_positions_sub;
        rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_ik_solver_foot_positions_pub;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_ik_solver_status_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_ik_solver_error_pub;

        bool m_gait_reset;
        int8_t m_gait_type;
        unsigned int m_n_joints;
        double m_dt;
        double m_convergence_threshold;
        bool m_early_stopping;
        long unsigned int m_early_stopping_threshold;
        Eigen::VectorXd m_current_joint_positions;
        Eigen::VectorXd m_desired_joint_positions;
        std::vector<march_shared_msgs::msg::IksFootPositions> m_ik_solver_foot_positions_buffer;
        march_shared_msgs::msg::IksFootPositions m_ik_solver_foot_positions_latest;
        bool m_ik_solver_foot_positions_received;
};

#endif // IK_SOLVER__IK_SOLVER_BUFFER_NODE_HPP_