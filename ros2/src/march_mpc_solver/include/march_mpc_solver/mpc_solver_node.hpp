/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_MPC_SOLVER__MPC_SOLVER_NODE_HPP_
#define MARCH_MPC_SOLVER__MPC_SOLVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "march_mpc_solver/mpc_solver.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"

#define LEFT_FOOT_ID 0
#define RIGHT_FOOT_ID 1

class MpcSolverNode : public rclcpp::Node {
public:
    MpcSolverNode();
    ~MpcSolverNode() = default;

private:
    void currentComCallback(march_shared_msgs::msg::CenterOfMass::SharedPtr);
    void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 
    void currentStanceFootCallback(std_msgs::msg::Int32::SharedPtr);
    void currentZmpCallback(geometry_msgs::msg::PointStamped::SharedPtr);
    void desiredFeetPositionsCallback(geometry_msgs::msg::PoseArray::SharedPtr);
    void estimatedFeetPositionsCallback(geometry_msgs::msg::PoseArray::SharedPtr);
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr);
    void timerCallback();

    void visualizeTrajectory();

    std::unique_ptr<MpcSolver> m_mpc_solver;

    double m_desired_previous_foot_x;
    double m_desired_previous_foot_y;

    uint8_t m_current_stance_leg;
    uint8_t m_next_stance_leg;

    geometry_msgs::msg::PoseArray::SharedPtr m_desired_footsteps;
    geometry_msgs::msg::PoseArray m_prev_des_footsteps;
    geometry_msgs::msg::PoseArray m_prev_foot_msg;
    std::vector<geometry_msgs::msg::Pose> m_inertial_foot_positions;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_final_feet_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_com_trajectory_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_com_visualizer_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_zmp_visualizer_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m__footstep_visualizer_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_current_shooting_node_publisher;

    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_stance_foot_subscriber;
    rclcpp::Subscription<march_shared_msgs::msg::CenterOfMass>::SharedPtr m_com_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_feet_pos_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_desired_steps_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_zmp_subscriber;
    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_subscriber; 

    rclcpp::TimerBase::SharedPtr m_solving_timer;

    ExoMode m_mode; 
};

#endif // MARCH_MPC_SOLVER__MPC_SOLVER_NODE_HPP_