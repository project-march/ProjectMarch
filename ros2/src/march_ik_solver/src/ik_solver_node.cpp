#include "march_ik_solver/ik_solver_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <chrono>
#include <functional>
#include <cstdlib>
#include <memory>

IKSolverNode::IKSolverNode()
    : Node("ik_solver", rclcpp::NodeOptions())
{
    // Initialize the node parameters.
    declare_parameter("dt", 0.05);
    declare_parameter("convergence_threshold", 0.0005);
    declare_parameter("max_iterations", 10);
    declare_parameter("joints.size", 0);
    declare_parameter("joints.names", std::vector<std::string>());
    declare_parameter("joints.limits.positions.upper", std::vector<double>());
    declare_parameter("joints.limits.positions.lower", std::vector<double>());
    declare_parameter("tasks.size", 0);
    declare_parameter("tasks.names", std::vector<std::string>());
    declare_parameter("tasks.m", std::vector<long int>());
    declare_parameter("tasks.n", std::vector<long int>());
    declare_parameter("tasks.kp", std::vector<double>());
    declare_parameter("tasks.kd", std::vector<double>());
    declare_parameter("tasks.ki", std::vector<double>());
    declare_parameter("tasks.damp_coeff", std::vector<double>());

    // Get the node parameters.
    m_convergence_threshold = get_parameter("convergence_threshold").as_double();
    m_max_iterations = get_parameter("max_iterations").as_int();
    m_joints_names = get_parameter("joints.names").as_string_array();
    double dt = get_parameter("dt").as_double();
    long unsigned int joints_size = get_parameter("joints.size").as_int();
    std::vector<double> joints_limits_upper = get_parameter("joints.limits.positions.upper").as_double_array();
    std::vector<double> joints_limits_lower = get_parameter("joints.limits.positions.lower").as_double_array();
    long unsigned int tasks_size = get_parameter("tasks.size").as_int();
    std::vector<std::string> tasks_names = get_parameter("tasks.names").as_string_array();
    std::vector<long int> tasks_m = get_parameter("tasks.m").as_integer_array();
    std::vector<long int> tasks_n = get_parameter("tasks.n").as_integer_array();
    std::vector<double> tasks_kp = get_parameter("tasks.kp").as_double_array();
    std::vector<double> tasks_kd = get_parameter("tasks.kd").as_double_array();
    std::vector<double> tasks_ki = get_parameter("tasks.ki").as_double_array();
    std::vector<double> tasks_damp_coeff = get_parameter("tasks.damp_coeff").as_double_array();

    RCLCPP_INFO(this->get_logger(), "dt: %f", dt);
    RCLCPP_INFO(this->get_logger(), "joints_size: %lu", joints_size);
    for (long unsigned int i = 0; i < m_joints_names.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "joint %d name: %s", i, m_joints_names[i].c_str());
        RCLCPP_INFO(this->get_logger(), "joints %d upper limit: %f", i, joints_limits_upper[i]);
        RCLCPP_INFO(this->get_logger(), "joints %d lower limit: %f", i, joints_limits_lower[i]);
    }
    RCLCPP_INFO(this->get_logger(), "tasks_size: %lu", tasks_size);
    for (long unsigned int i = 0; i < tasks_names.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "task %d name: %s", i, tasks_names[i].c_str());
        RCLCPP_INFO(this->get_logger(), "task %d m: %d", i, tasks_m[i]);
        RCLCPP_INFO(this->get_logger(), "task %d n: %d", i, tasks_n[i]);
        RCLCPP_INFO(this->get_logger(), "task %d kp: %f", i, tasks_kp[i]);
        RCLCPP_INFO(this->get_logger(), "task %d kd: %f", i, tasks_kd[i]);
        RCLCPP_INFO(this->get_logger(), "task %d ki: %f", i, tasks_ki[i]);
        RCLCPP_INFO(this->get_logger(), "task %d damp_coeff: %f", i, tasks_damp_coeff[i]);
    }

    // Initialize the state estimation message.
    m_state_estimation_msg = std::make_shared<march_shared_msgs::msg::StateEstimation>();
    RCLCPP_INFO(this->get_logger(), "State estimation message initialized.");

    // Configure the tasks.
    m_ik_solver.setCurrentJointPositionsPtr(&m_current_joint_positions, &m_joints_names);
    m_ik_solver.setDesiredJointPositionsPtr(&m_desired_joint_positions);
    m_ik_solver.setDesiredJointVelocitiesPtr(&m_desired_joint_velocities);
    RCLCPP_INFO(this->get_logger(), "IKSolver pointers set.");

    std::vector<Task> tasks;
    std::vector<std::string> task_nodes = {"left_ankle", "right_ankle"}; // TODO: Load this from a YAML file.
    for (long unsigned int i = 0; i < tasks_size; i++)
    {
        Task task = Task(i, tasks_names[i], tasks_m[i], tasks_n[i], task_nodes);
        task.setGainP(tasks_kp[i]);
        // task.setKd(tasks_kd[i]);
        // task.setKi(tasks_ki[i]);
        task.setDampingCoefficient(tasks_damp_coeff[i]);
        tasks.push_back(task);
    }
    RCLCPP_INFO(this->get_logger(), "Tasks configured.");

    m_ik_solver.setTasks(tasks);
    m_ik_solver.setNJoints(joints_size);
    m_ik_solver.setIntegralDtPtr(&m_desired_poses_dt);
    m_ik_solver.configureTasks(&m_desired_poses);
    m_ik_solver.setJointLimits(joints_limits_lower, joints_limits_upper);
    RCLCPP_INFO(this->get_logger(), "IKSolver configured.");

    // Configure the node.
    // exo_state_sub_ = this->create_subscription<march_shared_msgs::msg::ExoMode>(
    //     "current_state", 1, std::bind(&IKSolverNode::exoModeCallback, this, std::placeholders::_1));
    m_ik_solver_command_sub = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/input", 10, std::bind(&IKSolverNode::IksFootPositionsCallback, this, std::placeholders::_1));
    m_state_estimation_sub = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 10, std::bind(&IKSolverNode::stateEstimationCallback, this, std::placeholders::_1));
    m_joint_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 10); // TODO: Change queue.
    m_error_norm_pub = this->create_publisher<std_msgs::msg::Float64>(
        "ik_solver/buffer/error_norm", 10);

    RCLCPP_INFO(this->get_logger(), "IKSolverNode has been started.");

    // Configure previous joint trajectory point.
    // Eigen::VectorXd zeros = Eigen::VectorXd::Zero(joints_size);
    // std::vector<double> zeros_vector(zeros.data(), zeros.data() + zeros.size());
    // m_joint_trajectory_point_prev = trajectory_msgs::msg::JointTrajectoryPoint();
    // m_joint_trajectory_point_prev.positions = zeros_vector;
    // m_joint_trajectory_point_prev.velocities = zeros_vector;
    // m_joint_trajectory_point_prev.accelerations = zeros_vector;
    // m_joint_trajectory_point_prev.effort = zeros_vector;
    // m_joint_trajectory_point_prev.time_from_start.sec = 0;
    // m_joint_trajectory_point_prev.time_from_start.nanosec = 0;

    m_current_joint_positions = Eigen::VectorXd::Zero(joints_size);
    m_current_joint_velocities = Eigen::VectorXd::Zero(joints_size);
    m_desired_joint_positions = Eigen::VectorXd::Zero(joints_size);
    m_desired_joint_velocities = Eigen::VectorXd::Zero(joints_size);

    // // Configure exo state.
    // m_gait_reset = false;
    // m_gait_type = -1;
}

// void IKSolverNode::exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg)
// {
//     // RCLCPP_INFO(this->get_logger(), "ExoMode received.");
//     m_gait_type = msg->mode;
//     m_gait_reset = true;
//     // RCLCPP_INFO(this->get_logger(), "Gait type: %d", m_gait_type);
//     // RCLCPP_INFO(this->get_logger(), "Gait reset: %d", m_gait_reset);
// }

void IKSolverNode::IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "IKSolverCommand received.");
    m_desired_poses.clear();

    // TODO: Apply for loop after MVP.
    RCLCPP_INFO(this->get_logger(), "Processing the desired pose for both feet...");
    m_desired_poses_dt = msg->time_from_start.nanosec;
    geometry_msgs::msg::Point desired_pose_left = msg->left_foot_position;
    geometry_msgs::msg::Point desired_pose_right = msg->right_foot_position;
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
    desired_pose << desired_pose_left.x, desired_pose_left.y, desired_pose_left.z, desired_pose_right.x,
        desired_pose_right.y, desired_pose_right.z;
    m_desired_poses.push_back(desired_pose);

    RCLCPP_INFO(this->get_logger(), "Setting the current joint positions...");
    // // if (m_gait_reset)
    // if (msg->new_point)
    // {
    //     m_current_joint_positions = m_actual_joint_positions;
    //     m_current_joint_velocities = Eigen::VectorXd::Zero((int)m_actual_joint_velocities.size());
    //     m_current_joint_velocities = m_actual_joint_velocities;
    //     m_gait_reset = false;
    // }
    // else
    // {        
    //     m_current_joint_positions = m_desired_joint_positions;
    //     m_current_joint_velocities = m_desired_joint_velocities;
    // }

    // // Solve the IK problem to obtain the desired joint velocities.
    // // RCLCPP_INFO(this->get_logger(), "Solving the IK problem...");
    // m_desired_joint_velocities = m_ik_solver.solve();

    // // Print the desired joint velocities.
    // // RCLCPP_INFO(this->get_logger(), "Joint velocities: %f, %f, %f, %f, %f, %f, %f, %f", m_desired_joint_velocities(0),
    // //     m_desired_joint_velocities(1), m_desired_joint_velocities(2), m_desired_joint_velocities(3),
    // //     m_desired_joint_velocities(4), m_desired_joint_velocities(5), m_desired_joint_velocities(6),
    // //     m_desired_joint_velocities(7));

    // // Integrate the joint velocities to obtain the desired joint positions.
    // // RCLCPP_INFO(this->get_logger(), "Getting the current joint positions...");
    // // calculateDesiredJointStates();

    // // RCLCPP_INFO(this->get_logger(), "Integrating the joint velocities.");
    // m_desired_joint_positions = m_ik_solver.integrateJointVelocities();

    m_current_joint_positions = m_actual_joint_positions;
    m_current_joint_velocities = m_actual_joint_velocities;

    uint32_t iteration = 0;
    double error_norm = 0.0;

    while (this->now() - msg->header.stamp < rclcpp::Duration::from_seconds(0.05))
    {
        RCLCPP_INFO(this->get_logger(), "Iteration: %d", iteration);
        RCLCPP_INFO(this->get_logger(), "Getting the current joint positions...");
        
        if (iteration > 0)
        {
            m_current_joint_positions = m_desired_joint_positions;
            m_current_joint_velocities = m_desired_joint_velocities;
        }

        RCLCPP_INFO(this->get_logger(), "Solving the IK problem...");
        m_desired_joint_velocities = m_ik_solver.solve();
        RCLCPP_INFO(this->get_logger(), "Joint velocities: %f, %f, %f, %f, %f, %f, %f, %f", 
            m_desired_joint_velocities(0), m_desired_joint_velocities(1), m_desired_joint_velocities(2), m_desired_joint_velocities(3),
            m_desired_joint_velocities(4), m_desired_joint_velocities(5), m_desired_joint_velocities(6), m_desired_joint_velocities(7));

        RCLCPP_INFO(this->get_logger(), "Integrating the joint velocities...");
        m_desired_joint_positions = m_ik_solver.integrateJointVelocities();
        RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", 
            m_desired_joint_positions(0), m_desired_joint_positions(1), m_desired_joint_positions(2), m_desired_joint_positions(3),
            m_desired_joint_positions(4), m_desired_joint_positions(5), m_desired_joint_positions(6), m_desired_joint_positions(7));

        RCLCPP_INFO(this->get_logger(), "Calculating the error...");
        std::vector<double> error = m_ik_solver.getTasksError();
        RCLCPP_INFO(this->get_logger(), "Error: %f", error[0]);

        if (error[0] <= m_convergence_threshold)
        {
            RCLCPP_INFO(this->get_logger(), "Convergence reached.");
            break;
        }

        m_current_joint_positions = m_desired_joint_positions;
        // m_current_joint_velocities = m_desired_joint_velocities;

        iteration++;
        error_norm = error[0];
    }

    RCLCPP_INFO(this->get_logger(), "Number of iterations: %d", iteration);

    // Print the desired joint positions.
    // RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", m_desired_joint_positions(0),
    //     m_desired_joint_positions(1), m_desired_joint_positions(2), m_desired_joint_positions(3),
    //     m_desired_joint_positions(4), m_desired_joint_positions(5), m_desired_joint_positions(6),
    //     m_desired_joint_positions(7));

    // Publish the desired joint positions and velocities.
    publishJointTrajectory();

    // Publish the error norm.
    std_msgs::msg::Float64 error_norm_msg;
    error_norm_msg.data = error_norm;
    m_error_norm_pub->publish(error_norm_msg);
}

void IKSolverNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    m_actual_joint_positions = Eigen::VectorXd::Zero(msg->joint_state.position.size());
    m_actual_joint_velocities = Eigen::VectorXd::Zero(msg->joint_state.velocity.size());
    std::vector<long unsigned int> joint_ids = {3, 0, 1, 2, 7, 4, 5, 6};

    m_actual_joint_positions = Eigen::VectorXd::Zero((int) joint_ids.size());
    m_actual_joint_velocities = Eigen::VectorXd::Zero((int) joint_ids.size());
    for (unsigned int i = 0; i < joint_ids.size(); i++)
    {
        unsigned int joint_id = joint_ids[i];
        m_actual_joint_positions(i) = msg->joint_state.position[joint_id];
        m_actual_joint_velocities(i) = msg->joint_state.velocity[joint_id];
    }
}

void IKSolverNode::publishJointTrajectory()
{
    // Create the message to be published.
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = this->now();
    joint_trajectory_msg.joint_names = m_joints_names;
    
    // Create current trajectory point.
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_current;
    joint_trajectory_point_current.positions = std::vector<double>(m_actual_joint_positions.data(), m_actual_joint_positions.data() + m_actual_joint_positions.size());
    joint_trajectory_point_current.velocities = std::vector<double>(m_actual_joint_velocities.data(), m_actual_joint_velocities.data() + m_actual_joint_velocities.size());
    joint_trajectory_point_current.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0. };
    joint_trajectory_point_current.effort = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0. };
    joint_trajectory_point_current.time_from_start.sec = 0;
    joint_trajectory_point_current.time_from_start.nanosec = 0;
    joint_trajectory_msg.points.push_back(joint_trajectory_point_current);

    // Create desired trajectory point.
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_desired;
    joint_trajectory_point_desired.positions = std::vector<double>(m_desired_joint_positions.data(), m_desired_joint_positions.data() + m_desired_joint_positions.size());
    joint_trajectory_point_desired.velocities = std::vector<double>(m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size());
    joint_trajectory_point_desired.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0., 0. };
    joint_trajectory_point_desired.effort = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0. };
    joint_trajectory_point_desired.time_from_start.sec = 0;
    joint_trajectory_point_desired.time_from_start.nanosec = 50 * 1e6;
    joint_trajectory_msg.points.push_back(joint_trajectory_point_desired);

    // Publish the message.
    m_joint_trajectory_pub->publish(joint_trajectory_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSolverNode>());
    rclcpp::shutdown();
    return 0;
}
