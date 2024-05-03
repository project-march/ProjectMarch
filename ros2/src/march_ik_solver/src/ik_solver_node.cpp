/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/ik_solver_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "omp.h"
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <unordered_map>
#include <algorithm>

IKSolverNode::IKSolverNode()
    : Node("ik_solver", rclcpp::NodeOptions())
{
    m_ik_solver = std::make_unique<IKSolver>();
    configureIKSolverParameters();
    configureTasksParameters();
    configureIKSolutions();

    // Create the callback group and subscription options.
    m_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_subscription_options.callback_group = m_callback_group;

    // Create the subscriptions and publishers.
    m_ik_solver_command_sub = this->create_subscription<march_shared_msgs::msg::IksCommand>(
        "ik_solver/command", rclcpp::SensorDataQoS(),
        std::bind(&IKSolverNode::iksCommandCallback, this, std::placeholders::_1), m_subscription_options);
    m_ik_solver_foot_positions_sub = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/input", rclcpp::SensorDataQoS(),
        std::bind(&IKSolverNode::iksFootPositionsCallback, this, std::placeholders::_1), m_subscription_options);
    m_state_estimation_sub = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", rclcpp::SensorDataQoS(),
        std::bind(&IKSolverNode::stateEstimationCallback, this, std::placeholders::_1), m_subscription_options);
    m_joint_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "ik_solver/joint_trajectory", 10);
    m_iks_status_pub = this->create_publisher<march_shared_msgs::msg::IksStatus>("ik_solver/status", 10);
    m_desired_joint_positions_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("march_joint_position_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), "IKSolverNode has been started.");

    // Temporary
    m_first_step = true;
}

IKSolverNode::~IKSolverNode()
{
    RCLCPP_WARN(this->get_logger(), "IKSolverNode has been stopped.");
}

void IKSolverNode::iksCommandCallback(const march_shared_msgs::msg::IksCommand::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "IKSolver command received.");
    m_ik_solver->setTaskNames(msg->task_names);
}

void IKSolverNode::iksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
    /*
        Big TODO: Expand the stack of tasks
        - Separate Y-motion from "motion" task and create a new task for it.
            - This will allow the robot to stick to the desired Y-motion while the "motion" task is being solved.
    */
    std::vector<std::string> task_names = m_ik_solver->getTaskNames();
    if (task_names.empty()) {
        return;
    }

    // Vectorizing the desired tasks.
    std::unordered_map<std::string, Eigen::VectorXd> desired_tasks;
    // TODO: Magic number will be replaced in new ik_solver_buffer with ZMP.
    Eigen::VectorXd desired_motion = Eigen::VectorXd::Zero(6);
    desired_motion << 
        msg->left_foot_position.x, msg->left_foot_position.y, msg->left_foot_position.z,
        msg->right_foot_position.x, msg->right_foot_position.y, msg->right_foot_position.z;
    desired_tasks["motion"] = desired_motion;

    Eigen::VectorXd desired_stability = Eigen::VectorXd::Zero(2);
    desired_stability << m_x_stance_leg, m_y_stance_leg;
    desired_tasks["stability"] = desired_stability;

    Eigen::VectorXd desired_posture = Eigen::VectorXd::Zero(2);
    desired_tasks["posture"] = desired_posture;

    m_ik_solver->updateDesiredTasks(desired_tasks);
    m_ik_solver->updateCurrentJointState(m_actual_joint_positions, m_actual_joint_velocities);
    solveInverseKinematics(msg->header.stamp);
    publishDesiredJointPositions(); // Publish the desired joint positions to the hardware interface / mujoco writer.
    publishJointTrajectory();       // Publish the desired joint trajectory to the torque controller.
}

void IKSolverNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    m_actual_joint_positions.clear();
    m_actual_joint_velocities.clear();
    for (const auto& joint_name : m_joint_names) {
        auto it = std::find(msg->joint_state.name.begin(), msg->joint_state.name.end(), joint_name);
        if (it != msg->joint_state.name.end()) {
            std::size_t joint_id = std::distance(msg->joint_state.name.begin(), it);
            m_actual_joint_positions.push_back(msg->joint_state.position[joint_id]);
            m_actual_joint_velocities.push_back(msg->joint_state.velocity[joint_id]);
        }
    }
    m_ik_solver->updateWorldToBaseOrientation(
        msg->imu.orientation.w,
        msg->imu.orientation.x,
        msg->imu.orientation.y,
        msg->imu.orientation.z
    );

    if (m_first_step && (msg->next_stance_leg != 0b11)) {
        m_first_step = false;
    }

    Eigen::Vector3d stance_pos;
    // if (m_first_step) {
    // stance_pos.noalias()
    //     = Eigen::Quaterniond(msg->imu.orientation.w, msg->imu.orientation.x, msg->imu.orientation.y, msg->imu.orientation.z).normalized().toRotationMatrix().transpose()
    //         * Eigen::Vector3d(msg->body_sole_pose[1].position.x, msg->body_sole_pose[1].position.y, msg->body_sole_pose[1].position.z);
    // } else {
        m_ik_solver->updateCurrentStanceLeg(msg->current_stance_leg);
        if (msg->next_stance_leg == 0b01) {
            stance_pos.noalias()
                = Eigen::Quaterniond(msg->imu.orientation.w, msg->imu.orientation.x, msg->imu.orientation.y, msg->imu.orientation.z).normalized().toRotationMatrix().transpose()
                    * Eigen::Vector3d(msg->body_sole_pose[0].position.x, msg->body_sole_pose[0].position.y, msg->body_sole_pose[0].position.z);

        } else if (msg->next_stance_leg == 0b10) {
            stance_pos.noalias()
                = Eigen::Quaterniond(msg->imu.orientation.w, msg->imu.orientation.x, msg->imu.orientation.y, msg->imu.orientation.z).normalized().toRotationMatrix().transpose()
                    * Eigen::Vector3d(msg->body_sole_pose[1].position.x, msg->body_sole_pose[1].position.y, msg->body_sole_pose[1].position.z);
        } else {
            stance_pos.noalias() = (Eigen::Vector3d(msg->body_sole_pose[0].position.x, msg->body_sole_pose[0].position.y, msg->body_sole_pose[0].position.z) + Eigen::Vector3d(msg->body_sole_pose[1].position.x, msg->body_sole_pose[1].position.y, msg->body_sole_pose[1].position.z)) / 2;
            stance_pos.noalias()
                = Eigen::Quaterniond(msg->imu.orientation.w, msg->imu.orientation.x, msg->imu.orientation.y, msg->imu.orientation.z).normalized().toRotationMatrix().transpose()
                    * stance_pos;
        }
    // }
    m_x_stance_leg = stance_pos.x();
    m_y_stance_leg = stance_pos.y();
}

void IKSolverNode::publishJointTrajectory()
{
    // Create the message to be published.
    trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    joint_trajectory_msg->header.stamp = this->now();

    // Alphabetize the vector of joint names according to the alphabetical joint indices.
    joint_trajectory_msg->joint_names = m_joint_names;

    // Calculate the desired joint velocities.
    Eigen::VectorXd desired_joint_velocities;
    desired_joint_velocities.noalias()
        = (m_desired_joint_positions - Eigen::Map<const Eigen::VectorXd>(m_actual_joint_positions.data(), m_actual_joint_positions.size())) / m_state_estimator_time_offset;

    // // Publish the previous joint trajectory point.
    // joint_trajectory_msg->points.push_back(m_joint_trajectory_point_prev);

    // Create desired trajectory point.
    trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr joint_trajectory_point_desired = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
    joint_trajectory_point_desired->positions = std::vector<double>(
        m_desired_joint_positions.data(), m_desired_joint_positions.data() + m_desired_joint_positions.size());
    joint_trajectory_point_desired->velocities = std::vector<double>(
        desired_joint_velocities.data(), desired_joint_velocities.data() + desired_joint_velocities.size());
    joint_trajectory_point_desired->accelerations = createZeroVector();
    joint_trajectory_point_desired->effort = createZeroVector();
    joint_trajectory_point_desired->time_from_start.sec = 0;
    joint_trajectory_point_desired->time_from_start.nanosec = m_joint_trajectory_controller_period;
    joint_trajectory_msg->points.push_back(*joint_trajectory_point_desired);

    // Publish the message.
    m_joint_trajectory_pub->publish(*joint_trajectory_msg);

    // // Update the previous joint trajectory point.
    // updatePreviousJointTrajectoryPoint(*joint_trajectory_point_desired);
}

void IKSolverNode::publishDesiredJointPositions()
{
    std_msgs::msg::Float64MultiArray desired_joint_positions_msg;
    std::vector<double> desired_joint_positions = std::vector<double>(
        m_desired_joint_positions.data(), m_desired_joint_positions.data() + m_desired_joint_positions.size());
    for (const auto& idx : m_alphabetical_joint_indices) {
        // RCLCPP_INFO(this->get_logger(), "Joint index: %d", idx);
        desired_joint_positions_msg.data.push_back(desired_joint_positions[idx]);
    }
    m_desired_joint_positions_pub->publish(desired_joint_positions_msg);
}

void IKSolverNode::solveInverseKinematics(const rclcpp::Time& start_time)
{
    uint32_t iteration = 0;
    double best_error = 1e9;
    bool success = false;
    
    do {
        m_desired_joint_velocities = m_ik_solver->solveInverseKinematics();
        m_desired_joint_positions = m_ik_solver->integrateJointVelocities();
        best_error = m_ik_solver->getTasksError();

        if (best_error <= m_convergence_threshold) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Convergence reached.");
            success = true;
            break;
        }
        iteration++;
    } while (isWithinTimeWindow(start_time) && isWithinMaxIterations(iteration));
    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *get_clock(), 1000, "Iteration: %d, Error norm: %f", iteration, best_error);

    // Publish the IK status.
    march_shared_msgs::msg::IksStatus iks_status_msg = m_ik_solver->getIKStatus();
    iks_status_msg.header.stamp = this->now();
    iks_status_msg.iteration = iteration;
    iks_status_msg.success = success;
    m_iks_status_pub->publish(iks_status_msg);
}

void IKSolverNode::updatePreviousJointTrajectoryPoint(
    const trajectory_msgs::msg::JointTrajectoryPoint& joint_trajectory_point)
{
    m_joint_trajectory_point_prev = joint_trajectory_point;
    m_joint_trajectory_point_prev.time_from_start.sec = 0;
    m_joint_trajectory_point_prev.time_from_start.nanosec = 0;
}

void IKSolverNode::alphabetizeJointTrajectory(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    // Alphabetize the desired joint positions and velocities according to the joint names.
    std::vector<double> desired_joint_positions = msg->positions;
    std::vector<double> desired_joint_velocities = msg->velocities;

    // Sort the desired joint positions and velocities in the message according to the alphabetical joint indices.
    for (unsigned long int i = 0; i < m_joint_names.size(); i++) {
        msg->positions[i] = desired_joint_positions[m_alphabetical_joint_indices[i]];
        msg->velocities[i] = desired_joint_velocities[m_alphabetical_joint_indices[i]];
    }
}

void IKSolverNode::configureIKSolverParameters()
{
    declare_parameter("state_estimator_time_offset", 0.05);
    declare_parameter("joint_trajectory_controller_period", 0.05);
    declare_parameter("convergence_threshold", 0.0005);
    declare_parameter("max_iterations", 10);
    declare_parameter("integral_dt", 0.01);
    declare_parameter("joint.names", std::vector<std::string>());
    declare_parameter("joint.active", std::vector<bool>());
    declare_parameter("joint.limits.positions.upper", std::vector<double>());
    declare_parameter("joint.limits.positions.lower", std::vector<double>());
    declare_parameter("joint.limits.velocities.upper", std::vector<double>());
    declare_parameter("joint.limits.velocities.lower", std::vector<double>());
    declare_parameter("joint.limits.positions.soft", 0.0);

    m_state_estimator_time_offset = get_parameter("state_estimator_time_offset").as_double();
    m_convergence_threshold = get_parameter("convergence_threshold").as_double();
    m_max_iterations = get_parameter("max_iterations").as_int();
    m_joint_names = get_parameter("joint.names").as_string_array();
    m_ik_solver->setDt(get_parameter("integral_dt").as_double());

    // Convert double (in seconds) to uint64_t in (nanoseconds).
    double joint_trajectory_controller_period = get_parameter("joint_trajectory_controller_period").as_double();
    m_joint_trajectory_controller_period = (uint64_t)(joint_trajectory_controller_period * 1e9);

    std::vector<double> joint_position_limits_upper = get_parameter("joint.limits.positions.upper").as_double_array();
    std::vector<double> joint_position_limits_lower = get_parameter("joint.limits.positions.lower").as_double_array();
    std::vector<double> joint_velocity_limits_upper = get_parameter("joint.limits.velocities.upper").as_double_array();
    std::vector<double> joint_velocity_limits_lower = get_parameter("joint.limits.velocities.lower").as_double_array();

    // Apply soft limits to the joint position limits.
    RCLCPP_INFO(this->get_logger(), "Aplying soft limits to the joint position limits.");
    double soft_limit = get_parameter("joint.limits.positions.soft").as_double();
    if (soft_limit < 0.0) {
        RCLCPP_WARN(this->get_logger(), "Soft limit must be greater than or equal to zero. Setting to zero.");
        soft_limit = 0.0;
    }
    for (unsigned long int i = 0; i < m_joint_names.size(); i++) {
        joint_position_limits_upper[i] -= soft_limit;
        joint_position_limits_lower[i] += soft_limit;
        RCLCPP_INFO(this->get_logger(), "Joint name: %s, Upper limit: %f, Lower limit: %f",
            m_joint_names[i].c_str(), joint_position_limits_upper[i], joint_position_limits_lower[i]);
    }

    // Set the joint positions and velocities limits in the IK solver.
    m_ik_solver->setJointConfigurations(m_joint_names, 
        joint_position_limits_lower, joint_position_limits_upper,
        joint_velocity_limits_lower, joint_velocity_limits_upper);

    // Get an array of active joints and store the active joint names.
    std::vector<bool> joint_active = get_parameter("joint.active").as_bool_array();
    std::vector<std::string> active_joint_names;
    for (unsigned long int i = 0; i < m_joint_names.size(); i++) {
        if (joint_active[i]) {
            active_joint_names.push_back(m_joint_names[i]);
            RCLCPP_INFO(this->get_logger(), "Active joint: %s", m_joint_names[i].c_str());
        }
    }

    // Store the joint indices in alphabetical order according to the total joint names.
    m_alphabetical_joint_indices.clear();
    m_alphabetical_joint_indices.resize(m_joint_names.size());
    std::iota(m_alphabetical_joint_indices.begin(), m_alphabetical_joint_indices.end(), 0);
    std::sort(m_alphabetical_joint_indices.begin(), m_alphabetical_joint_indices.end(),
        [&](const unsigned int& a, const unsigned int& b) {
            return m_joint_names[a] < m_joint_names[b];
        });

    // Filter the joint indices according to the active joint names.
    m_alphabetical_joint_indices.erase(
        std::remove_if(m_alphabetical_joint_indices.begin(), m_alphabetical_joint_indices.end(),
            [&](const unsigned int& idx) {
                return std::find(active_joint_names.begin(), active_joint_names.end(), m_joint_names[idx]) == active_joint_names.end();
            }),
        m_alphabetical_joint_indices.end());
    
    // Create the alphabetical joint names, and print the joint indices and names.
    m_joint_names_alphabetical.clear();
    RCLCPP_INFO(this->get_logger(), "Alphabetical joint indices:");
    for (const auto& joint_index : m_alphabetical_joint_indices) {
        m_joint_names_alphabetical.push_back(m_joint_names[joint_index]);
        RCLCPP_INFO(this->get_logger(), "Joint index: %d, Joint name: %s", joint_index, m_joint_names[joint_index].c_str());
    }
}

void IKSolverNode::configureTasksParameters()
{
    declare_parameter("task.names", std::vector<std::string>());
    std::vector<std::string> task_names = get_parameter("task.names").as_string_array();
    m_ik_solver->setTaskNames(task_names);

    std::unordered_map<std::string, std::vector<double>> task_gains_p, task_gains_d, task_gains_i;
    std::unordered_map<std::string, double> task_damp_coeffs;
    std::unordered_map<std::string, double> task_convergence_thresholds;

    for (const auto& task_name : task_names) {
        RCLCPP_INFO(this->get_logger(), "Configuring task name: %s", task_name.c_str());
        declare_parameter("task." + task_name + ".kp", std::vector<double>());
        declare_parameter("task." + task_name + ".kd", std::vector<double>());
        declare_parameter("task." + task_name + ".ki", std::vector<double>());
        declare_parameter("task." + task_name + ".damp_coeff", 0.0);
        declare_parameter("task." + task_name + ".convergence_threshold", 0.0);

        task_gains_p[task_name] = get_parameter("task." + task_name + ".kp").as_double_array();compute PID error
        task_damp_coeffs[task_name] = get_parameter("task." + task_name + ".damp_coeff").as_double();
        task_convergence_thresholds[task_name] = get_parameter("task." + task_name + ".convergence_threshold").as_double();
    }

    m_ik_solver->createTask(task_gains_p, task_gains_d, task_gains_i, task_damp_coeffs, task_convergence_thresholds);
}

void IKSolverNode::configureIKSolutions()
{
    m_joint_trajectory_point_prev = trajectory_msgs::msg::JointTrajectoryPoint();
    m_joint_trajectory_point_prev.positions = createZeroVector();
    m_joint_trajectory_point_prev.velocities = createZeroVector();
    m_joint_trajectory_point_prev.accelerations = createZeroVector();
    m_joint_trajectory_point_prev.effort = createZeroVector();
    m_joint_trajectory_point_prev.time_from_start.sec = 0;
    m_joint_trajectory_point_prev.time_from_start.nanosec = 0;

    m_desired_joint_positions = Eigen::VectorXd::Zero(m_joint_names.size());
    m_desired_joint_velocities = Eigen::VectorXd::Zero(m_joint_names.size());
}

bool IKSolverNode::isWithinTimeWindow(const rclcpp::Time& time_stamp)
{
    return (this->now() - time_stamp) < rclcpp::Duration::from_seconds(m_state_estimator_time_offset);
}

bool IKSolverNode::isWithinMaxIterations(const unsigned int& iterations)
{
    return iterations < m_max_iterations;
}

std::vector<double> IKSolverNode::createZeroVector()
{
    std::vector<double> zero_vector(m_joint_names.size(), 0.0);
    return zero_vector;
}

int main(int argc, char** argv)
{
    Eigen::initParallel();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSolverNode>());
    rclcpp::shutdown();
    return 0;
}
