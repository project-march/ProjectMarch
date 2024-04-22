/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include <functional>
#include <future>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "march_ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/iks_status.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode();
    ~IKSolverNode();

private:
    void iksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void publishJointTrajectory();
    void publishErrorNorm(const double& error_norm);
    void publishIterations(const unsigned int& iterations);

    void solveInverseKinematics(const rclcpp::Time& start_time);
    void updatePreviousJointTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& joint_trajectory_point);
    void alphabetizeJointTrajectory(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr joint_trajectory_point);
    void configureIKSolverParameters();
    void configureTasksParameters();
    void configureIKSolutions();
    bool isWithinTimeWindow(const rclcpp::Time& time_stamp);
    bool isWithinMaxIterations(const unsigned int& iterations);
    std::vector<double> createZeroVector();

    std::unique_ptr<IKSolver> m_ik_solver;
    uint64_t m_joint_trajectory_controller_period;
    double m_state_estimator_time_offset;
    double m_convergence_threshold;
    unsigned int m_max_iterations;

    std::vector<std::string> m_joint_names;
    std::vector<std::string> m_joint_names_alphabetical;
    std::vector<unsigned int> m_alphabetical_joint_indices;
    std::vector<double> m_actual_joint_positions;
    std::vector<double> m_actual_joint_velocities;
    Eigen::VectorXd m_desired_joint_positions;
    Eigen::VectorXd m_desired_joint_velocities;
    trajectory_msgs::msg::JointTrajectoryPoint m_joint_trajectory_point_prev;

    // ROS2 communication
    rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_ik_solver_command_sub;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_pub;
    rclcpp::Publisher<march_shared_msgs::msg::IksStatus>::SharedPtr m_iks_status_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_error_norm_pub;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr m_iterations_pub;

    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::SubscriptionOptions m_subscription_options;
};

#endif // IK_SOLVER__IK_SOLVER_NODE_HPP_