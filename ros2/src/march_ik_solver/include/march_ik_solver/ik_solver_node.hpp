#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include <functional>
#include <future>
#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "march_ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/exo_state.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include "rclcpp/rclcpp.hpp"

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode();
    ~IKSolverNode() = default;

private:
    void exoStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg);
    void IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
    // void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    // void publishJointTrajectory(bool reset);
    void publishJointTrajectory();
    void calculateDesiredJointStates();
    void currentJointPositionsCallback(
        const rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture future);
    std::vector<Eigen::VectorXd> calculateError();
    trajectory_msgs::msg::JointTrajectory convertToJointTrajectoryMsg();
    std::vector<std::array<double,2>> getJointLimits();

    IKSolver m_ik_solver; // TODO: make this a pointer using std::unique_ptr<IKSolver> ik_solver_;
    double m_convergence_threshold;
    uint32_t m_max_iterations;
    std::vector<std::string> m_joints_names;
    Eigen::VectorXd m_current_joint_positions;
    Eigen::VectorXd m_current_joint_velocities;
    Eigen::VectorXd m_actual_joint_positions;
    Eigen::VectorXd m_actual_joint_velocities;
    Eigen::VectorXd m_desired_joint_positions;
    Eigen::VectorXd m_desired_joint_velocities;
    std::vector<Eigen::VectorXd> m_desired_poses;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> m_joint_trajectory_points;
    trajectory_msgs::msg::JointTrajectoryPoint m_joint_trajectory_point_prev;
    uint32_t m_desired_poses_dt;
    bool m_gait_reset;
    int8_t m_gait_type;

    march_shared_msgs::msg::StateEstimation::SharedPtr m_state_estimation_msg;

    // rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<march_shared_msgs::msg::ExoState>::SharedPtr m_exo_state_sub;
    rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_ik_solver_command_sub;
    // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_pub;
    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_desired_pose_pub;
    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_actual_pose_pub;

    rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr m_current_joint_positions_client;
    rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture m_current_joint_positions_future;
    march_shared_msgs::srv::GetCurrentJointPositions::Request::SharedPtr m_current_joint_positions_request;

};

#endif // IK_SOLVER__IK_SOLVER_NODE_HPP_