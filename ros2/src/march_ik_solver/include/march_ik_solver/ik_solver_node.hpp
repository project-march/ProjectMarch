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
#include "march_shared_msgs/msg/iks_command.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/iks_status.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode();
    ~IKSolverNode();

private:
    void iksCommandCallback(const march_shared_msgs::msg::IksCommand::SharedPtr msg);
    void iksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void publishJointTrajectory();
    void publishDesiredJointPositions();

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
    bool m_has_solution;

    std::vector<std::string> m_joint_names;
    std::vector<std::string> m_joint_names_alphabetical;
    std::vector<unsigned int> m_alphabetical_joint_indices;
    std::vector<double> m_actual_joint_positions;
    std::vector<double> m_actual_joint_velocities;
    Eigen::VectorXd m_desired_joint_positions;
    Eigen::VectorXd m_desired_joint_velocities;
    Eigen::Quaterniond m_current_world_to_base_orientation;
    trajectory_msgs::msg::JointTrajectoryPoint m_joint_trajectory_point_prev;

    // ROS2 communication
    rclcpp::Subscription<march_shared_msgs::msg::IksCommand>::SharedPtr m_ik_solver_command_sub;
    rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_ik_solver_foot_positions_sub;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_pub;
    rclcpp::Publisher<march_shared_msgs::msg::IksStatus>::SharedPtr m_iks_status_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_desired_joint_positions_pub;

    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::SubscriptionOptions m_subscription_options;
};

#endif // IK_SOLVER__IK_SOLVER_NODE_HPP_