#include "march_ik_solver/ik_solver_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <unordered_map>

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
    m_ik_solver_command_sub = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/input", rclcpp::SensorDataQoS(), 
        std::bind(&IKSolverNode::iksFootPositionsCallback, this, std::placeholders::_1), m_subscription_options);
    m_state_estimation_sub = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", rclcpp::SensorDataQoS(), 
        std::bind(&IKSolverNode::stateEstimationCallback, this, std::placeholders::_1), m_subscription_options);
    m_joint_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10);
    m_error_norm_pub = this->create_publisher<std_msgs::msg::Float64>("ik_solver/error", 10);

    RCLCPP_DEBUG(this->get_logger(), "IKSolverNode has been started.");
}

void IKSolverNode::iksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
    // Vectorizing the desired tasks.
    std::unordered_map<std::string, Eigen::VectorXd> desired_tasks;
    // TODO: Magic number will be replaced in new ik_solver_buffer with ZMP.
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
    desired_pose << 
        msg->left_foot_position.x, msg->left_foot_position.y, msg->left_foot_position.z,
        msg->right_foot_position.x, msg->right_foot_position.y, msg->right_foot_position.z;
    desired_tasks["motion"] = desired_pose;
    m_ik_solver->updateDesiredTasks(desired_tasks);

    m_ik_solver->updateCurrentJointState(m_actual_joint_positions, m_actual_joint_velocities);
    solveInverseKinematics(msg->header.stamp);
    publishJointTrajectory();
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
}

void IKSolverNode::publishJointTrajectory()
{
    // Create the message to be published.
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = this->now();
    joint_trajectory_msg.joint_names = m_joint_names;

    // Push back the previous trajectory point.
    joint_trajectory_msg.points.push_back(m_joint_trajectory_point_prev);

    // Create desired trajectory point.
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_desired;
    joint_trajectory_point_desired.positions = std::vector<double>(
        m_desired_joint_positions.data(), m_desired_joint_positions.data() + m_desired_joint_positions.size());
    joint_trajectory_point_desired.velocities = std::vector<double>(
        m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size());
    joint_trajectory_point_desired.accelerations = createZeroVector();
    joint_trajectory_point_desired.effort = createZeroVector();
    joint_trajectory_point_desired.time_from_start.sec = 0;
    joint_trajectory_point_desired.time_from_start.nanosec = m_joint_trajectory_controller_period;
    joint_trajectory_msg.points.push_back(joint_trajectory_point_desired);

    // Publish the message.
    m_joint_trajectory_pub->publish(joint_trajectory_msg);

    // Update the previous trajectory point.
    joint_trajectory_point_desired.time_from_start.sec = 0;
    joint_trajectory_point_desired.time_from_start.nanosec = 0;
    m_joint_trajectory_point_prev = joint_trajectory_point_desired;
}

void IKSolverNode::solveInverseKinematics(const rclcpp::Time& start_time)
{
    uint32_t iteration = 0;
    double best_error = 1e9;
    do {
        Eigen::VectorXd desired_joint_velocities = m_ik_solver->solveInverseKinematics();
        Eigen::VectorXd desired_joint_positions = m_ik_solver->integrateJointVelocities();
        std::vector<double> error = m_ik_solver->getTasksError();
        if (error[0] < best_error)
        {
            m_desired_joint_velocities = desired_joint_velocities;
            m_desired_joint_positions = desired_joint_positions;
            best_error = error[0];
        }
        if (best_error <= m_convergence_threshold) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Convergence reached.");
            break;
        }
        iteration++;
    }
    while (isWithinTimeWindow(start_time) && isWithinMaxIterations(iteration));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 2000, "Iteration: %d, Error norm: %f", iteration, best_error);

    // Publish the error norm.
    std_msgs::msg::Float64 error_norm_msg;
    error_norm_msg.data = best_error;
    m_error_norm_pub->publish(error_norm_msg);
}

void IKSolverNode::configureIKSolverParameters()
{
    declare_parameter("state_estimator_time_offset", 0.05);
    declare_parameter("joint_trajectory_controller_period", 0.05);
    declare_parameter("convergence_threshold", 0.0005);
    declare_parameter("max_iterations", 10);
    declare_parameter("dt", 0.01);
    declare_parameter("joint.names", std::vector<std::string>());
    declare_parameter("joint.limits.positions.upper", std::vector<double>());
    declare_parameter("joint.limits.positions.lower", std::vector<double>());

    m_state_estimator_time_offset = get_parameter("state_estimator_time_offset").as_double();
    m_convergence_threshold = get_parameter("convergence_threshold").as_double();
    m_max_iterations = get_parameter("max_iterations").as_int();
    m_joint_names = get_parameter("joint.names").as_string_array();
    m_ik_solver->setDt(get_parameter("dt").as_double());

    // Convert double (in seconds) to uint64_t in (nanoseconds).
    double joint_trajectory_controller_period = get_parameter("joint_trajectory_controller_period").as_double();
    m_joint_trajectory_controller_period = (uint64_t)(joint_trajectory_controller_period * 1e9);

    std::vector<double> joint_limits_upper = get_parameter("joint.limits.positions.upper").as_double_array();
    std::vector<double> joint_limits_lower = get_parameter("joint.limits.positions.lower").as_double_array();

    m_ik_solver->setJointConfigurations(m_joint_names, joint_limits_lower, joint_limits_upper);
}

void IKSolverNode::configureTasksParameters()
{
    declare_parameter("task_names", std::vector<std::string>());
    std::vector<std::string> task_names = get_parameter("task_names").as_string_array();
    m_ik_solver->setTaskNames(task_names);

    for (const auto& task_name : task_names) {
        RCLCPP_DEBUG(this->get_logger(), "Task name: %s", task_name.c_str());
        declare_parameter(task_name + ".nodes", std::vector<std::string>());
        declare_parameter(task_name + ".m", 0);
        declare_parameter(task_name + ".n", 0);
        declare_parameter(task_name + ".kp", std::vector<double>());
        declare_parameter(task_name + ".kd", std::vector<double>());
        declare_parameter(task_name + ".ki", std::vector<double>());
        declare_parameter(task_name + ".damp_coeff", 0.0);

        std::vector<std::string> nodes = get_parameter(task_name + ".nodes").as_string_array();
        long unsigned int task_dim = get_parameter(task_name + ".m").as_int();
        long unsigned int workspace_dim = get_parameter(task_name + ".n").as_int();
        std::vector<double> kp = get_parameter(task_name + ".kp").as_double_array();
        std::vector<double> kd = get_parameter(task_name + ".kd").as_double_array();
        std::vector<double> ki = get_parameter(task_name + ".ki").as_double_array();
        double damp_coeff = get_parameter(task_name + ".damp_coeff").as_double();

        m_ik_solver->createTask(task_name, nodes, task_dim, workspace_dim, kp, kd, ki, damp_coeff);
    }
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
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto node = std::make_shared<IKSolverNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
