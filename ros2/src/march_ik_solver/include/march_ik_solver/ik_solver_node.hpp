#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include <functional>
#include <future>
#include <string>
#include <vector>

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "march_ik_solver/ik_solver.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include "rclcpp/rclcpp.hpp"

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode();
    ~IKSolverNode() = default;

private:
    void exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);
    void IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
    // void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    // void publishJointTrajectory(bool reset);
    void publishJointTrajectory();
    void publishJointTrajectoryControllerState();

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
    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_state_sub;
    rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_ik_solver_command_sub;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_pub;
    // rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr m_joint_trajectory_controller_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_error_norm_pub;

};

#endif // IK_SOLVER__IK_SOLVER_NODE_HPP_