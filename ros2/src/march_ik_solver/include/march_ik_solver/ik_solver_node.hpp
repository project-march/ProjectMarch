/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "march_ik_solver/ik_solver.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <functional>
#include <future>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "march_ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/msg/iks_command.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/iks_status.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

typedef message_filters::sync_policies::ApproximateTime<march_shared_msgs::msg::IksFootPositions, geometry_msgs::msg::PoseStamped> IKSynchronizer;
typedef message_filters::sync_policies::ApproximateTime<march_shared_msgs::msg::ExoMode, march_shared_msgs::msg::StateEstimation> SyncPolicy_NewGait;

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode();
    ~IKSolverNode();

private:
    void iksSyncCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr foot_positions_msg,
        const geometry_msgs::msg::PoseStamped::SharedPtr com_pose_msg);
    void clockCallback(const std_msgs::msg::Header::SharedPtr msg);
    void iksCommandCallback(const march_shared_msgs::msg::IksCommand::SharedPtr msg);
    void desiredFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
    void desiredComPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void newGaitCallback(const march_shared_msgs::msg::ExoMode::SharedPtr exo_mode_msg, 
        const march_shared_msgs::msg::StateEstimation::SharedPtr state_estimation_msg);
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
    Eigen::Vector2d m_desired_com_position;
    Eigen::Matrix3d m_current_world_to_base_orientation;
    trajectory_msgs::msg::JointTrajectoryPoint m_joint_trajectory_point_prev;

    // ROS2 communication
    // message_filters::Subscriber<march_shared_msgs::msg::IksFootPositions> m_desired_foot_positions_sub;
    // message_filters::Subscriber<geometry_msgs::msg::PoseStamped> m_desired_com_pose_sub;
    // std::shared_ptr<message_filters::Synchronizer<IKSynchronizer>> m_ik_sync_sub;
    message_filters::Subscriber<march_shared_msgs::msg::ExoMode> m_new_gait_exo_mode_sub;
    message_filters::Subscriber<march_shared_msgs::msg::StateEstimation> m_new_gait_state_estimation_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy_NewGait>> m_new_gait_sync;

    rclcpp::Subscription<march_shared_msgs::msg::IksCommand>::SharedPtr m_ik_solver_command_sub;
    rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_desired_foot_positions_sub;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_desired_com_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr m_clock_sub;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_pub;
    rclcpp::Publisher<march_shared_msgs::msg::IksStatus>::SharedPtr m_iks_status_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_desired_joint_positions_pub;

    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::SubscriptionOptions m_subscription_options;
};

#endif // IK_SOLVER__IK_SOLVER_NODE_HPP_