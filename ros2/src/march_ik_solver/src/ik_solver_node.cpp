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

    // Configure the tasks.
    m_ik_solver.setCurrentJointPositionsPtr(&m_current_joint_positions, &m_joints_names);
    m_ik_solver.setDesiredJointPositionsPtr(&m_desired_joint_positions);
    m_ik_solver.setDesiredJointVelocitiesPtr(&m_desired_joint_velocities);
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
    m_ik_solver.setTasks(tasks);
    m_ik_solver.setNJoints(joints_size);
    m_ik_solver.setIntegralDtPtr(&m_desired_poses_dt);
    m_ik_solver.configureTasks(&m_desired_poses);
    m_ik_solver.setJointLimits(joints_limits_lower, joints_limits_upper);

    // Configure the node.
    // exo_state_sub_ = this->create_subscription<march_shared_msgs::msg::ExoMode>(
    //     "current_state", 1, std::bind(&IKSolverNode::exoModeCallback, this, std::placeholders::_1));
    m_ik_solver_command_sub = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/input", 10, std::bind(&IKSolverNode::IksFootPositionsCallback, this, std::placeholders::_1));
    // joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    //     "joint_states", 1, std::bind(&IKSolverNode::jointStateCallback, this, std::placeholders::_1));
    m_state_estimation_sub = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 10, std::bind(&IKSolverNode::stateEstimationCallback, this, std::placeholders::_1));
    m_joint_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 100); // TODO: Change queue.
    // desired_pose_pub_ = this->create_publisher<march_shared_msgs::msg::IksFootPositions>("desired_pose", 10);
    // actual_pose_pub_ = this->create_publisher<march_shared_msgs::msg::IksFootPositions>("actual_pose", 10);
    // current_joint_positions_client_
    //     = this->create_client<march_shared_msgs::srv::GetCurrentJointPositions>("get_current_joint_positions");

    // Configure IK solver.
    // std::vector<uint8_t> tasks_m = m_ik_solver.getTasksM();
    // for (auto task_m : tasks_m)
    // {
    //     Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(task_m);
    //     m_desired_poses.push_back(desired_pose);
    // }

    RCLCPP_INFO(this->get_logger(), "IKSolverNode has been started.");

    // Configure previous joint trajectory point.
    Eigen::VectorXd zeros = Eigen::VectorXd::Zero(joints_size);
    std::vector<double> zeros_vector(zeros.data(), zeros.data() + zeros.size());
    m_joint_trajectory_point_prev = trajectory_msgs::msg::JointTrajectoryPoint();
    m_joint_trajectory_point_prev.positions = zeros_vector;
    m_joint_trajectory_point_prev.velocities = zeros_vector;
    m_joint_trajectory_point_prev.accelerations = zeros_vector;
    m_joint_trajectory_point_prev.effort = zeros_vector;
    m_joint_trajectory_point_prev.time_from_start.sec = 0;
    m_joint_trajectory_point_prev.time_from_start.nanosec = 0;

    m_current_joint_positions = Eigen::VectorXd::Zero(joints_size);
    m_current_joint_velocities = Eigen::VectorXd::Zero(joints_size);
    m_desired_joint_positions = Eigen::VectorXd::Zero(joints_size);
    m_desired_joint_velocities = Eigen::VectorXd::Zero(joints_size);

    // Configure exo state.
    m_gait_reset = false;
    m_gait_type = -1;
}

void IKSolverNode::exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "ExoMode received.");
    m_gait_type = msg->mode;
    m_gait_reset = true;
    // RCLCPP_INFO(this->get_logger(), "Gait type: %d", m_gait_type);
    // RCLCPP_INFO(this->get_logger(), "Gait reset: %d", m_gait_reset);
}

void IKSolverNode::IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "IKSolverCommand received.");
    m_desired_poses.clear();

    // TODO: Apply for loop after MVP.
    // RCLCPP_INFO(this->get_logger(), "Processing the desired pose for both feet...");
    m_desired_poses_dt = msg->time_from_start.nanosec;
    geometry_msgs::msg::Point desired_pose_left = msg->left_foot_position;
    geometry_msgs::msg::Point desired_pose_right = msg->right_foot_position;
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
    desired_pose << desired_pose_left.x, desired_pose_left.y, desired_pose_left.z, desired_pose_right.x,
        desired_pose_right.y, desired_pose_right.z;
    m_desired_poses.push_back(desired_pose);

    // RCLCPP_INFO(this->get_logger(), "Setting the current joint positions...");
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
    m_current_joint_velocities = Eigen::VectorXd::Zero((int)m_desired_joint_velocities.size());

    uint32_t iteration = 0;
    while (iteration < m_max_iterations)
    {
        // RCLCPP_INFO(this->get_logger(), "Iteration: %d", iteration);
        // RCLCPP_INFO(this->get_logger(), "Getting the current joint positions...");
        // calculateDesiredJointStates();

        // RCLCPP_INFO(this->get_logger(), "Solving the IK problem...");
        m_desired_joint_velocities = m_ik_solver.solve();

        // RCLCPP_INFO(this->get_logger(), "Integrating the joint velocities.");
        m_desired_joint_positions = m_ik_solver.integrateJointVelocities();

        // RCLCPP_INFO(this->get_logger(), "Calculating the error...");
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
    }

    // RCLCPP_INFO(this->get_logger(), "Number of iterations: %d", iteration);


    // // Publish desired pose.
    // // RCLCPP_INFO(this->get_logger(), "Publishing the desired pose.");
    // std::vector<double> new_desired_pose = m_ik_solver.getPose(&m_desired_joint_positions);
    // march_shared_msgs::msg::IksFootPositions desired_pose_msg;
    // desired_pose_msg.header.stamp = this->now();
    // desired_pose_msg.left_foot_position.x = new_desired_pose[0];
    // desired_pose_msg.left_foot_position.y = new_desired_pose[1];
    // desired_pose_msg.left_foot_position.z = new_desired_pose[2];
    // desired_pose_msg.right_foot_position.x = new_desired_pose[3];
    // desired_pose_msg.right_foot_position.y = new_desired_pose[4];
    // desired_pose_msg.right_foot_position.z = new_desired_pose[5];
    // desired_pose_msg.time_from_start.sec = 0;
    // desired_pose_msg.time_from_start.nanosec = msg->time_from_start.nanosec;

    // desired_pose_pub_->publish(desired_pose_msg);

    // Print the desired joint positions.
    // RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", m_desired_joint_positions(0),
    //     m_desired_joint_positions(1), m_desired_joint_positions(2), m_desired_joint_positions(3),
    //     m_desired_joint_positions(4), m_desired_joint_positions(5), m_desired_joint_positions(6),
    //     m_desired_joint_positions(7));

    // Publish the desired joint positions and velocities.
    // RCLCPP_INFO(this->get_logger(), "Publishing the desired joint positions and velocities.");
    // publishJointTrajectory(msg->new_point);
    publishJointTrajectory();
}

// void IKSolverNode::RobotStateCallback(const march_shared_msgs::msg::RobotState::SharedPtr msg)
// {
//     // Update the current joint positions.
//     Eigen::VectorXd current_joint_positions = Eigen::Map<Eigen::VectorXd>(msg->joint_pos.data(), msg->joint_pos.size());
//     m_current_joint_positions = current_joint_positions;

//     // Print the current joint positions.
//     RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
//         m_current_joint_positions(0), m_current_joint_positions(1), m_current_joint_positions(2),
//         m_current_joint_positions(3), m_current_joint_positions(4), m_current_joint_positions(5),
//         m_current_joint_positions(6), m_current_joint_positions(7));
// }
// void IKSolverNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
// {
//     // RCLCPP_INFO(this->get_logger(), "JointState received.");
//     // Update the current joint positions.
//     m_actual_joint_positions = Eigen::VectorXd::Zero((int) msg->position.size());
//     m_actual_joint_positions(0) = msg->position[3];
//     m_actual_joint_positions(1) = msg->position[0];
//     m_actual_joint_positions(2) = msg->position[1];
//     m_actual_joint_positions(3) = msg->position[2];
//     m_actual_joint_positions(4) = msg->position[7];
//     m_actual_joint_positions(5) = msg->position[4];
//     m_actual_joint_positions(6) = msg->position[5];
//     m_actual_joint_positions(7) = msg->position[6];
//     // m_actual_joint_positions = Eigen::Map<Eigen::VectorXd>(msg->position.data(), msg->position.size());

//     // Update the current joint velocities.
//     m_actual_joint_velocities = Eigen::VectorXd::Zero((int)msg->velocity.size());
//     m_actual_joint_velocities(0) = msg->velocity[3];
//     m_actual_joint_velocities(1) = msg->velocity[0];
//     m_actual_joint_velocities(2) = msg->velocity[1];
//     m_actual_joint_velocities(3) = msg->velocity[2];
//     m_actual_joint_velocities(4) = msg->velocity[7];
//     m_actual_joint_velocities(5) = msg->velocity[4];
//     m_actual_joint_velocities(6) = msg->velocity[5];
//     m_actual_joint_velocities(7) = msg->velocity[6];

//     // Publish actual pose.
//     // RCLCPP_INFO(this->get_logger(), "Publishing the actual pose.");
//     std::vector<double> new_actual_pose = m_ik_solver.getPose(&m_actual_joint_positions);
//     march_shared_msgs::msg::IksFootPositions actual_pose_msg;
//     actual_pose_msg.header.stamp = this->now();
//     actual_pose_msg.left_foot_position.x = new_actual_pose[0];
//     actual_pose_msg.left_foot_position.y = new_actual_pose[1];
//     actual_pose_msg.left_foot_position.z = new_actual_pose[2];
//     actual_pose_msg.right_foot_position.x = new_actual_pose[3];
//     actual_pose_msg.right_foot_position.y = new_actual_pose[4];
//     actual_pose_msg.right_foot_position.z = new_actual_pose[5];
//     actual_pose_msg.time_from_start.sec = 0;
//     actual_pose_msg.time_from_start.nanosec = m_desired_poses;
//     actual_pose_pub_->publish(actual_pose_msg);    

//     // m_actual_joint_velocities = Eigen::Map<Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size());

//     // // Print the current joint positions.
//     // RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
//     //     m_current_joint_positions(0), m_current_joint_positions(1), m_current_joint_positions(2),
//     //     m_current_joint_positions(3), m_current_joint_positions(4), m_current_joint_positions(5),
//     //     m_current_joint_positions(6), m_current_joint_positions(7));

//     // // Print the current joint velocities.
//     // RCLCPP_INFO(this->get_logger(), "Current joint velocities: %f, %f, %f, %f, %f, %f, %f, %f",
//     //     m_current_joint_velocities(0), m_current_joint_velocities(1), m_current_joint_velocities(2),
//     //     m_current_joint_velocities(3), m_current_joint_velocities(4), m_current_joint_velocities(5),
//     //     m_current_joint_velocities(6), m_current_joint_velocities(7));
// }

void IKSolverNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "StateEstimation received.");

    m_state_estimation_msg = msg;

    // TODO: Optimize this function using a map.
    // TODO: Do this only once because the difference in the order of the joints never changes.

    // RCLCPP_INFO(this->get_logger(), "Joint names: %s, %s, %s, %s, %s, %s, %s, %s",
    //     msg->joint_state.name[0].c_str(), msg->joint_state.name[1].c_str(), msg->joint_state.name[2].c_str(), msg->joint_state.name[3].c_str(),
    //     msg->joint_state.name[4].c_str(), msg->joint_state.name[5].c_str(), msg->joint_state.name[6].c_str(), msg->joint_state.name[7].c_str());
    // RCLCPP_INFO(this->get_logger(), "Joint angles: %f, %f, %f, %f, %f, %f, %f, %f",
    //     msg->joint_state.position[0], msg->joint_state.position[1], msg->joint_state.position[2], msg->joint_state.position[3],
    //     msg->joint_state.position[4], msg->joint_state.position[5], msg->joint_state.position[6], msg->joint_state.position[7]);

    std::vector<long unsigned int> joints_ids;
    for (long unsigned int i = 0; i < m_joints_names.size(); i++)
    {
        // RCLCPP_INFO(this->get_logger(), "Joint name: %s", m_joints_names[i].c_str());
        for (long unsigned int j = 0; j < msg->joint_state.name.size(); j++)
        {
            // RCLCPP_INFO(this->get_logger(), "Joint state name: %s", msg->joint_state.name[j].c_str());
            if (m_joints_names[i] == msg->joint_state.name[j])
            {
                joints_ids.push_back(j);
                break;
            }
        }
    }

    // Update the current joint positions.
    m_actual_joint_positions = Eigen::VectorXd::Zero((int) m_joints_names.size());
    m_actual_joint_velocities = Eigen::VectorXd::Zero((int) m_joints_names.size());
    for (long unsigned int i = 0; i < m_joints_names.size(); i++)
    {
        m_actual_joint_positions(i) = msg->joint_state.position[joints_ids[i]];
        m_actual_joint_velocities(i) = msg->joint_state.velocity[joints_ids[i]];
    }
    
    // RCLCPP_INFO(this->get_logger(), "Joint names: %s, %s, %s, %s, %s, %s, %s, %s",
    //     m_joints_names[0].c_str(), m_joints_names[1].c_str(), m_joints_names[2].c_str(), m_joints_names[3].c_str(),
    //     m_joints_names[4].c_str(), m_joints_names[5].c_str(), m_joints_names[6].c_str(), m_joints_names[7].c_str());
    // RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", m_actual_joint_positions(0),
    //     m_actual_joint_positions(1), m_actual_joint_positions(2), m_actual_joint_positions(3),
    //     m_actual_joint_positions(4), m_actual_joint_positions(5), m_actual_joint_positions(6),
    //     m_actual_joint_positions(7));

    // Publish actual pose.
    // RCLCPP_INFO(this->get_logger(), "Publishing the actual pose.");

    // RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
    //     m_current_joint_positions(0), m_current_joint_positions(1), m_current_joint_positions(2),
    //     m_current_joint_positions(3), m_current_joint_positions(4), m_current_joint_positions(5),
    //     m_current_joint_positions(6), m_current_joint_positions(7));
}

// void IKSolverNode::publishJointTrajectory(bool reset)
// {
//     // // Create the message to be published.
//     // trajectory_msgs::msg::JointTrajectory joint_trajectory_msg
//     //     = convertToJointTrajectoryMsg();

//     // // Publish the message.
//     // m_joint_trajectory_pub->publish(joint_trajectory_msg);

//     if (reset)
//     {
//         if (joint_trajectory_points_.size() > 0)
//         {
//             // Publish the current joint positions and velocities.
//             trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
//             joint_trajectory_msg.header.stamp = this->now();
//             joint_trajectory_msg.joint_names = m_joints_names;
//             joint_trajectory_msg.points = joint_trajectory_points_;
//             m_joint_trajectory_pub->publish(joint_trajectory_msg);
//             joint_trajectory_points_.clear();
//         }

//         // Add new point.
//         trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_current;
//         joint_trajectory_point_current.positions = std::vector<double>(m_current_joint_positions.data(), m_current_joint_positions.data() + m_current_joint_positions.size());
//         joint_trajectory_point_current.velocities = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //std::vector<double>(m_current_joint_velocities.data(), m_current_joint_velocities.data() + m_current_joint_velocities.size());
//         joint_trajectory_point_current.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//         joint_trajectory_point_current.effort = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//         joint_trajectory_point_current.time_from_start.sec = 0;
//         joint_trajectory_point_current.time_from_start.nanosec = 0;
//         joint_trajectory_points_.push_back(joint_trajectory_point_current);
//         // RCLCPP_INFO(this->get_logger(), "Publishing the current joint positions and velocities.");
//         // RCLCPP_INFO(this->get_logger(), "Time: %d", joint_trajectory_point_current.time_from_start.nanosec);
//     }
//     else
//     {
//         // // Determine if the desired joint positions and velocities are reached.
//         // std::vector<double> desired_pose = m_ik_solver.getPose(&m_desired_joint_positions);
//         // Eigen::VectorXd desired_pose_eigen = Eigen::Map<Eigen::VectorXd>(desired_pose.data(), desired_pose.size());
//         // Eigen::VectorXd error = m_desired_poses[0] - desired_pose_eigen;
//         // double error_norm = error.norm();
//         // RCLCPP_INFO(this->get_logger(), "Error norm: %f", error_norm);
//         // if (error_norm < m_convergence_threshold)
//         // {
//         //     RCLCPP_INFO(this->get_logger(), "Desired joint positions and velocities reached.");
//         //     return;
//         // }

//         // Add new point.
//         uint32_t previous_time = joint_trajectory_points_.back().time_from_start.nanosec;
//         trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
//         joint_trajectory_point.positions = std::vector<double>(m_desired_joint_positions.data(), m_desired_joint_positions.data() + m_desired_joint_positions.size());
//         joint_trajectory_point.velocities = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //std::vector<double>(m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size());
//         joint_trajectory_point.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//         joint_trajectory_point.effort = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//         joint_trajectory_point.time_from_start.sec = 0;
//         joint_trajectory_point.time_from_start.nanosec = previous_time + m_desired_poses;
//         joint_trajectory_points_.push_back(joint_trajectory_point);
//         // RCLCPP_INFO(this->get_logger(), "Publishing the desired joint positions and velocities.");
//         // RCLCPP_INFO(this->get_logger(), "Time: %d", joint_trajectory_point.time_from_start.nanosec);
//     }
// }

// std::vector<Eigen::VectorXd> IKSolverNode::calculateError()
// {
//     // Define a vector of Eigen::VectorXd to store the error.
//     std::vector<Eigen::VectorXd> error;

//     // Calculate error.
//     Eigen::VectorXd error_joint_positions_ = m_desired_joint_positions - m_current_joint_positions;
//     Eigen::VectorXd error_joint_velocities_ = m_desired_joint_velocities - m_current_joint_velocities;

//     // Push the error to the vector.
//     error.push_back(error_joint_positions_);
//     error.push_back(error_joint_velocities_);

//     return error;
// }

void IKSolverNode::publishJointTrajectory()
{
    // Create the message to be published.
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = this->now();
    joint_trajectory_msg.joint_names = m_joints_names;
    
    // Create current trajectory point.
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_current;
    joint_trajectory_point_current.positions = std::vector<double>(m_actual_joint_positions.data(), m_actual_joint_positions.data() + m_actual_joint_positions.size());
    joint_trajectory_point_current.velocities = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //std::vector<double>(m_current_joint_velocities.data(), m_current_joint_velocities.data() + m_current_joint_velocities.size());
    joint_trajectory_point_current.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0. };
    joint_trajectory_point_current.effort = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0. };
    joint_trajectory_point_current.time_from_start.sec = 0;
    joint_trajectory_point_current.time_from_start.nanosec = 0;
    joint_trajectory_msg.points.push_back(joint_trajectory_point_current);

    // Create desired trajectory point.
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_desired;
    joint_trajectory_point_desired.positions = std::vector<double>(m_desired_joint_positions.data(), m_desired_joint_positions.data() + m_desired_joint_positions.size());
    joint_trajectory_point_desired.velocities = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //std::vector<double>(m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size());
    joint_trajectory_point_desired.accelerations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0., 0. };
    joint_trajectory_point_desired.effort = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0., 0. };
    joint_trajectory_point_desired.time_from_start.sec = 0;
    joint_trajectory_point_desired.time_from_start.nanosec = 50 * 1e6;
    joint_trajectory_msg.points.push_back(joint_trajectory_point_desired);

    // Publish the message.
    m_joint_trajectory_pub->publish(joint_trajectory_msg);
}

// trajectory_msgs::msg::JointTrajectory IKSolverNode::convertToJointTrajectoryMsg()
// {
//     // // Calculate the error.
//     // std::vector<Eigen::VectorXd> error_joints = calculateError();

//     // Convert Eigen::VectorXd to a vector of doubles.
//     Eigen::VectorXd zeros = Eigen::VectorXd::Zero(8);
//     std::vector<double> zeros_vector(zeros.data(), zeros.data() + zeros.size());
//     std::vector<double> desired_positions_vector(
//         m_desired_joint_positions.data(), m_desired_joint_positions.data() + m_desired_joint_positions.size());
//     std::vector<double> desired_velocities_vector(
//         m_desired_joint_velocities.data(), m_desired_joint_velocities.data() + m_desired_joint_velocities.size());
//     std::vector<double> current_positions_vector(
//         m_current_joint_positions.data(), m_current_joint_positions.data() + m_current_joint_positions.size());
//     std::vector<double> current_velocities_vector(
//         m_current_joint_velocities.data(), m_current_joint_velocities.data() + m_current_joint_velocities.size());
//     // std::vector<double> error_positions_vector(error_joints[0].data(), error_joints[0].data() + error_joints[0].size());
//     // std::vector<double> error_velocities_vector(
//     //     error_joints[1].data(), error_joints[1].data() + error_joints[1].size());

//     // Create the message.
//     trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
//     joint_trajectory_msg.header.stamp = this->now();
//     joint_trajectory_msg.joint_names = m_joints_names;
//     // joint_trajectory_msg.points.push_back(m_joint_trajectory_point_prev);

//     // for (long unsigned int i = 0; i < joint_trajectory_msg.joint_names.size(); i++)
//     // {
//     //     trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
//     //     joint_trajectory_point.positions = { desired_positions_vector[i] };
//     //     joint_trajectory_point.velocities = { desired_velocities_vector[i] };
//     //     joint_trajectory_point.accelerations = { 0.0 };
//     //     joint_trajectory_point.effort = { 0.0 };
//     //     joint_trajectory_point.time_from_start = rclcpp::Duration(0.0);
//     //     joint_trajectory_msg.points.push_back(joint_trajectory_point);
//     // }

//     if (m_gait_reset)
//     {
//         // Publish the current joint positions and velocities.
//         trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_current;
//         joint_trajectory_point_current.positions = current_positions_vector;
//         joint_trajectory_point_current.velocities = current_velocities_vector;
//         joint_trajectory_point_current.accelerations = zeros_vector;
//         joint_trajectory_point_current.effort = zeros_vector;
//         joint_trajectory_point_current.time_from_start.sec = 0;
//         joint_trajectory_point_current.time_from_start.nanosec = 0;
//         joint_trajectory_msg.points.push_back(joint_trajectory_point_current);
//         m_gait_reset = false;
//     }
//     else
//     {
//         // Publish the previous joint positions and velocities.
//         m_joint_trajectory_point_prev.time_from_start.nanosec = 0;
//         joint_trajectory_msg.points.push_back(m_joint_trajectory_point_prev);
//     }

//     // Publish the desired joint positions and velocities.
//     trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point_desired;
//     joint_trajectory_point_desired.positions = desired_positions_vector;
//     joint_trajectory_point_desired.velocities = desired_velocities_vector;
//     joint_trajectory_point_desired.accelerations = zeros_vector;
//     joint_trajectory_point_desired.effort = zeros_vector;
//     joint_trajectory_point_desired.time_from_start.sec = 0;
//     joint_trajectory_point_desired.time_from_start.nanosec = (uint32_t) (m_desired_poses * 1e6);
//     joint_trajectory_msg.points.push_back(joint_trajectory_point_desired);

//     m_joint_trajectory_point_prev = joint_trajectory_point_desired;
//     m_joint_trajectory_point_prev.time_from_start.nanosec = 0;

//     // DEBUG
//     // Publish the robot state.
//     sensor_msgs::msg::JointState robot_state_msg;
//     robot_state_msg.header.stamp = this->now();
//     robot_state_msg.name = m_joints_names;
//     robot_state_msg.position = desired_positions_vector;
//     robot_state_msg.velocity = desired_velocities_vector;
//     robot_state_msg.effort = zeros_vector;
//     robot_state_pub_->publish(robot_state_msg);

//     return joint_trajectory_msg;
// }

// void IKSolverNode::calculateDesiredJointStates()
// {
//     // Get the current joint positions from the state estimation node.
//     current_joint_positions_request_ = std::make_shared<march_shared_msgs::srv::GetCurrentJointPositions::Request>();
//     current_joint_positions_future_
//         = current_joint_positions_client_->async_send_request(current_joint_positions_request_,
//             std::bind(&IKSolverNode::currentJointPositionsCallback, this, std::placeholders::_1));
// }

// void IKSolverNode::currentJointPositionsCallback(
//     const rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture future)
// {
//     // RCLCPP_INFO(this->get_logger(), "Receiving response from /get_current_joint_positions...");
//     if (future.get()->status) {
//         RCLCPP_INFO(this->get_logger(), "Current joint positions received.");
//         // if (m_gait_reset)
//         // {
//             // m_current_joint_positions = Eigen::Map<Eigen::VectorXd>(
//             //     current_joint_positions_future_.get()->joint_positions.data(),
//             //     current_joint_positions_future_.get()->joint_positions.size());
//         // }
//         // else
//         // {
//         //     // TODO: Check if this is necessary.
//         //     m_current_joint_positions = m_desired_joint_positions;
//         // }

//         // m_current_joint_velocities
//         //     = Eigen::Map<Eigen::VectorXd>(current_joint_positions_future_.get()->joint_velocities.data(),
//         //         current_joint_positions_future_.get()->joint_velocities.size());

//         // RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
//         //     m_current_joint_positions(0), m_current_joint_positions(1), m_current_joint_positions(2),
//         //     m_current_joint_positions(3), m_current_joint_positions(4), m_current_joint_positions(5),
//         //     m_current_joint_positions(6), m_current_joint_positions(7));

//         // RCLCPP_INFO(this->get_logger(), "Integrating the joint velocities.");
//         m_desired_joint_positions = m_ik_solver.integrateJointVelocities();

//         // Print the desired joint positions.
//         // RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", m_desired_joint_positions(0),
//         //     m_desired_joint_positions(1), m_desired_joint_positions(2), m_desired_joint_positions(3),
//         //     m_desired_joint_positions(4), m_desired_joint_positions(5), m_desired_joint_positions(6),
//         //     m_desired_joint_positions(7));

//         // Publish the desired joint positions and velocities.
//         // RCLCPP_INFO(this->get_logger(), "Publishing the desired joint positions and velocities.");
//         publishJointTrajectory();
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to get current joint positions.");
//     }
// }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSolverNode>());
    rclcpp::shutdown();
    return 0;
}
