/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_mpc_solver/mpc_solver_node.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

MpcSolverNode::MpcSolverNode(): Node("mpc_solver")
    , m_desired_previous_foot_x(0.0)
    , m_desired_previous_foot_y(0.33)
{
    m_mpc_solver = std::make_unique<MpcSolver>();

    m_com_trajectory_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("com_trajectory", 10);
    m_final_feet_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("final_feet_position", 10);
    m_com_visualizer_publisher = this->create_publisher<nav_msgs::msg::Path>("com_visualization_trajectory", 10);
    m_zmp_visualizer_publisher = this->create_publisher<nav_msgs::msg::Path>("zmp_visualization_trajectory", 10);
    m__footstep_visualizer_publisher
        = this->create_publisher<visualization_msgs::msg::Marker>("footsteps_visualization", 100);
    m_current_shooting_node_publisher = this->create_publisher<std_msgs::msg::Int32>("current_shooting_node", 10);

    m_com_subscriber = this->create_subscription<march_shared_msgs::msg::CenterOfMass>(
        "robot_com_position", 10, std::bind(&MpcSolverNode::currentComCallback, this, std::placeholders::_1));
    m_desired_steps_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "desired_footsteps", 10, std::bind(&MpcSolverNode::desiredFeetPositionsCallback, this, std::placeholders::_1));
    m_feet_pos_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "est_foot_position", 10, std::bind(&MpcSolverNode::estimatedFeetPositionsCallback, this, std::placeholders::_1));
    m_zmp_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "robot_zmp_position", 10, std::bind(&MpcSolverNode::currentZmpCallback, this, std::placeholders::_1));
    m_stance_foot_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "current_stance_foot", 10, std::bind(&MpcSolverNode::currentStanceFootCallback, this, std::placeholders::_1));
    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&MpcSolverNode::currentModeCallback, this, std::placeholders::_1)); 
    m_state_estimation_subscriber = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 10, std::bind(&MpcSolverNode::stateEstimationCallback, this, std::placeholders::_1));
    geometry_msgs::msg::Pose prev_foot_pose_container;

    m_prev_foot_msg.header.frame_id = "R_heel";
    prev_foot_pose_container.position.x = 0.0;
    prev_foot_pose_container.position.y = 0.0;
    prev_foot_pose_container.position.z = 0.0;
    m_prev_foot_msg.poses.push_back(prev_foot_pose_container);

    m_solving_timer = this->create_wall_timer(std::chrono::milliseconds(50), 
        std::bind(&MpcSolverNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "MPC Solver node has been started.");
}

void MpcSolverNode::currentComCallback(march_shared_msgs::msg::CenterOfMass::SharedPtr msg)
{
    m_mpc_solver->set_current_com(msg->position.point.x, msg->position.point.y, msg->velocity.x, msg->velocity.y);
    m_mpc_solver->set_com_height(msg->position.point.z);
}

void MpcSolverNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg)
{
    m_mode = (exoMode) msg->mode; 
}

void MpcSolverNode::currentStanceFootCallback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_mpc_solver->set_current_stance_foot(msg->data);
}

void MpcSolverNode::currentZmpCallback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_mpc_solver->set_current_zmp(msg->point.x, msg->point.y);
}

void MpcSolverNode::desiredFeetPositionsCallback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // CHANGE THIS, NEED AN EXTRA TOPIC. THIS ONE IS CONNECTED TO THE FOOTSTEP PLANNER, NEED ONE FROM STATE ESTIMATION
    // OR SOMETHING FOR CURRENT FEET POSITIONS.
    m_desired_footsteps = msg;
    m_mpc_solver->set_candidate_footsteps(m_desired_footsteps);
    m_mpc_solver->set_reference_stepsize(m_mpc_solver->get_candidate_footsteps());
}

void MpcSolverNode::estimatedFeetPositionsCallback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // msg->poses[0] is the left foot
    // msg->poses[1] is the right foot
    if ((m_mpc_solver->get_current_stance_foot() == -1)
        || (m_mpc_solver->get_current_stance_foot() == 1
            && m_mpc_solver->get_m_current_shooting_node().data == 0)) { // if stance foot is the left foot
        // only change the desired previous footsteps when current shooting node is 1 and the footstep changes
        if (abs(m_desired_previous_foot_y - msg->poses[LEFT_FOOT_ID].position.y) > 10e-2
            && m_mpc_solver->get_m_current_shooting_node().data == 1) {
            m_desired_previous_foot_x = msg->poses[LEFT_FOOT_ID].position.x;
            m_desired_previous_foot_y = msg->poses[LEFT_FOOT_ID].position.y;
        }
    }
    if (m_mpc_solver->get_current_stance_foot() == 1
        || (m_mpc_solver->get_current_stance_foot() == -1 && m_mpc_solver->get_m_current_shooting_node().data == 0)) {
        if (abs(m_desired_previous_foot_y - msg->poses[RIGHT_FOOT_ID].position.y) > 10e-2
            && m_mpc_solver->get_m_current_shooting_node().data == 1) {
            m_desired_previous_foot_x = msg->poses[RIGHT_FOOT_ID].position.x;
            m_desired_previous_foot_y = msg->poses[RIGHT_FOOT_ID].position.y;
        }
    }
    m_mpc_solver->set_previous_foot(m_desired_previous_foot_x, m_desired_previous_foot_y);
}

void MpcSolverNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) 
{
    m_inertial_foot_positions = msg->world_foot_pose;
    m_current_stance_leg = msg->current_stance_leg;
    m_next_stance_leg = msg->next_stance_leg;
}

void MpcSolverNode::timerCallback()
{
    if (m_mode != exoMode::VariableWalk){ 
        return;
    }

    if (!(m_desired_footsteps)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for input from footstep planner");
        return;
    }

    if (*m_desired_footsteps != m_prev_des_footsteps) {
        m_mpc_solver->reset_to_double_stance();
    }

    m_prev_des_footsteps = *m_desired_footsteps;
    m_mpc_solver->update_current_foot();
    m_mpc_solver->set_current_state();
    int solver_status = m_mpc_solver->solve_step();

    if (solver_status != 0) {
        RCLCPP_WARN(this->get_logger(), "Could not find a solution. exited with status %i", solver_status);
        return;
    }

    m_current_shooting_node_publisher->publish(m_mpc_solver->get_m_current_shooting_node());
    m_mpc_solver->update_current_shooting_node();
    visualizeTrajectory();
}

void MpcSolverNode::visualizeTrajectory()
{
    auto com_msg = geometry_msgs::msg::PoseArray();
    com_msg.header.stamp = this->get_clock()->now();
    com_msg.header.frame_id = "R_heel";

    auto foot_msg = geometry_msgs::msg::PoseArray();
    foot_msg.header.stamp = this->get_clock()->now();
    foot_msg.header.frame_id = "R_heel";

    geometry_msgs::msg::Pose pose_container;

    visualization_msgs::msg::Marker current_footsteps_marker;
    current_footsteps_marker.type = 8;
    current_footsteps_marker.header.frame_id = "R_heel";
    current_footsteps_marker.id = 0;

    visualization_msgs::msg::Marker previous_footsteps_marker;
    previous_footsteps_marker.type = 8;
    previous_footsteps_marker.header.frame_id = "R_heel";
    previous_footsteps_marker.id = 1;

    geometry_msgs::msg::Point marker_container;

    nav_msgs::msg::Path com_path;
    com_path.header.frame_id = "R_heel";

    nav_msgs::msg::Path zmp_path;
    zmp_path.header.frame_id = "R_heel";

    geometry_msgs::msg::PoseStamped com_path_wrapper;
    com_path_wrapper.header.frame_id = "R_heel";

    geometry_msgs::msg::PoseStamped zmp_path_wrapper;
    zmp_path_wrapper.header.frame_id = "R_heel";

    std::array<double, NX* ZMP_PENDULUM_ODE_N>* trajectory_pointer = m_mpc_solver->get_state_trajectory();

    for (int i = 0; i < (ZMP_PENDULUM_ODE_N); i++) {
        pose_container.position.x = (*trajectory_pointer)[(i * NX + 0)];
        pose_container.position.y = (*trajectory_pointer)[(i * NX + 3)];
        pose_container.position.z = m_mpc_solver->get_com_height();
        com_msg.poses.push_back(pose_container);
        com_path_wrapper.pose = pose_container;
        com_path.poses.push_back(com_path_wrapper);

        // Visualize the ZMP trajectory
        pose_container.position.x = (*trajectory_pointer)[(i * NX + 2)];
        pose_container.position.y = (*trajectory_pointer)[(i * NX + 5)];
        pose_container.position.z = 0 ; // m_mpc_solver->get_com_height();
        zmp_path_wrapper.pose = pose_container;
        zmp_path.poses.push_back(zmp_path_wrapper);

        // The feet
        pose_container.position.x = (*trajectory_pointer)[(i * NX + 6)];
        pose_container.position.y = (*trajectory_pointer)[(i * NX + 8)];
        foot_msg.poses.push_back(pose_container);
        if (i == ZMP_PENDULUM_ODE_N - 1) {
            continue;
        }

        marker_container.x = (*trajectory_pointer)[(i * NX + 6)];
        marker_container.y = (*trajectory_pointer)[(i * NX + 8)];
        marker_container.z = 0.0;
        current_footsteps_marker.points.push_back(marker_container);

        marker_container.x = (*trajectory_pointer)[(i * NX + 7)];
        marker_container.y = (*trajectory_pointer)[(i * NX + 9)];
        marker_container.z = m_mpc_solver->get_com_height() / 2;
        previous_footsteps_marker.points.push_back(marker_container);
    };

    current_footsteps_marker.action = 0;
    current_footsteps_marker.frame_locked = 1;
    current_footsteps_marker.scale.x = 0.1;
    current_footsteps_marker.scale.y = 0.1;
    current_footsteps_marker.scale.z = 0.1;
    current_footsteps_marker.pose.position.x = 0.0;
    current_footsteps_marker.pose.position.y = 0.0;
    current_footsteps_marker.pose.position.z = 0.0;
    current_footsteps_marker.pose.orientation.x = 0.0;
    current_footsteps_marker.pose.orientation.y = 0.0;
    current_footsteps_marker.pose.orientation.z = 0.0;
    current_footsteps_marker.pose.orientation.w = 1.0;
    current_footsteps_marker.ns = "current_footstep_visualization";
    current_footsteps_marker.lifetime.sec = 1;
    current_footsteps_marker.color.b = 1.0;
    current_footsteps_marker.color.a = 0.7;

    previous_footsteps_marker.action = 0;
    previous_footsteps_marker.frame_locked = 1;
    previous_footsteps_marker.scale.x = 0.1;
    previous_footsteps_marker.scale.y = 0.1;
    previous_footsteps_marker.scale.z = 0.1;
    previous_footsteps_marker.pose.position.x = 0.0;
    previous_footsteps_marker.pose.position.y = 0.0;
    previous_footsteps_marker.pose.position.z = 0.0;
    previous_footsteps_marker.pose.orientation.x = 0.0;
    previous_footsteps_marker.pose.orientation.y = 0.0;
    previous_footsteps_marker.pose.orientation.z = 0.0;
    previous_footsteps_marker.pose.orientation.w = 1.0;
    previous_footsteps_marker.ns = "previous_footstep_visualization";
    previous_footsteps_marker.lifetime.sec = 1;
    previous_footsteps_marker.color.r = 1.0;
    previous_footsteps_marker.color.a = 1.0;

    m__footstep_visualizer_publisher->publish(previous_footsteps_marker);
    m__footstep_visualizer_publisher->publish(current_footsteps_marker);
    m_com_visualizer_publisher->publish(com_path);
    m_zmp_visualizer_publisher->publish(zmp_path);
    m_com_trajectory_publisher->publish(com_msg);

    if (abs(foot_msg.poses[0].position.y - m_prev_foot_msg.poses[0].position.y) > 10e-2)
    {
        // TODO: Move this into separate function
        // only publish when foot is changing
        m_final_feet_publisher->publish(foot_msg);
        m_prev_foot_msg = foot_msg;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MpcSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
