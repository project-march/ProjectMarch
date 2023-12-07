#include "march_ik_solver/ik_solver_node.hpp"
using std::placeholders::_1;

IKSolverNode::IKSolverNode()
    : Node("ik_solver")
{
    ik_solver_command_sub_ = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
        "iks_foot_positions", 10, std::bind(&IKSolverNode::IksFootPositionsCallback, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&IKSolverNode::jointStateCallback, this, std::placeholders::_1));
    joint_trajectory_controller_state_pub_ = this->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "joint_trajectory_controller/state", 1); // TODO: Change queue.
    current_joint_positions_client_
        = this->create_client<march_shared_msgs::srv::GetCurrentJointPositions>("get_current_joint_positions");

    // Configure IK solver.
    std::vector<uint8_t> tasks_m = ik_solver_.getTasksM();
    for (auto task_m : tasks_m) {
        Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(task_m);
        desired_poses_.push_back(desired_pose);
    }
    ik_solver_.setCurrentJointPositionsPtr(&current_joint_positions_);
    ik_solver_.setDesiredJointPositionsPtr(&desired_joint_positions_);
    ik_solver_.setDesiredJointVelocitiesPtr(&desired_joint_velocities_);

    RCLCPP_INFO(this->get_logger(), "IKSolverNode has been started.");
}

void IKSolverNode::IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "IKSolverCommand received.");
    std::vector<Eigen::VectorXd> desired_poses;

    // TODO: Apply for loop after MVP.
    RCLCPP_INFO(this->get_logger(), "Processing the desired pose for both feet...");
    geometry_msgs::msg::Point desired_pose_left = msg->left_foot_position;
    geometry_msgs::msg::Point desired_pose_right = msg->right_foot_position;
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
    desired_pose << desired_pose_left.x, desired_pose_left.y, desired_pose_left.z, desired_pose_right.x,
        desired_pose_right.y, desired_pose_right.z;
    desired_poses.push_back(desired_pose);

    // Solve the IK problem to obtain the desired joint velocities.
    RCLCPP_INFO(this->get_logger(), "Solving the IK problem...");
    desired_joint_velocities_ = ik_solver_.solve(desired_poses);

    // Print the desired joint velocities.
    RCLCPP_INFO(this->get_logger(), "Joint velocities: %f, %f, %f, %f, %f, %f, %f, %f", desired_joint_velocities_(0),
        desired_joint_velocities_(1), desired_joint_velocities_(2), desired_joint_velocities_(3),
        desired_joint_velocities_(4), desired_joint_velocities_(5), desired_joint_velocities_(6),
        desired_joint_velocities_(7));

    // Integrate the joint velocities to obtain the desired joint positions.
    RCLCPP_INFO(this->get_logger(), "Getting the current joint positions...");
    // calculateDesiredJointStates();

    RCLCPP_INFO(this->get_logger(), "Integrating the joint velocities.");
    desired_joint_positions_ = ik_solver_.integrateJointVelocities();

    // // Print the desired joint positions.
    // RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", desired_joint_positions_(0),
    //     desired_joint_positions_(1), desired_joint_positions_(2), desired_joint_positions_(3),
    //     desired_joint_positions_(4), desired_joint_positions_(5), desired_joint_positions_(6),
    //     desired_joint_positions_(7));

    // Publish the desired joint positions and velocities.
    RCLCPP_INFO(this->get_logger(), "Publishing the desired joint positions and velocities.");
    publishJointTrajectoryControllerState();    
}

// void IKSolverNode::RobotStateCallback(const march_shared_msgs::msg::RobotState::SharedPtr msg)
// {
//     // Update the current joint positions.
//     Eigen::VectorXd current_joint_positions = Eigen::Map<Eigen::VectorXd>(msg->joint_pos.data(), msg->joint_pos.size());
//     current_joint_positions_ = current_joint_positions;

//     // Print the current joint positions.
//     RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
//         current_joint_positions_(0), current_joint_positions_(1), current_joint_positions_(2),
//         current_joint_positions_(3), current_joint_positions_(4), current_joint_positions_(5),
//         current_joint_positions_(6), current_joint_positions_(7));
// }
void IKSolverNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Update the current joint positions.
    current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(msg->position.data(), msg->position.size());

    // Update the current joint velocities.
    current_joint_velocities_ = Eigen::Map<Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size());

    // // Print the current joint positions.
    // RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
    //     current_joint_positions_(0), current_joint_positions_(1), current_joint_positions_(2),
    //     current_joint_positions_(3), current_joint_positions_(4), current_joint_positions_(5),
    //     current_joint_positions_(6), current_joint_positions_(7));

    // // Print the current joint velocities.
    // RCLCPP_INFO(this->get_logger(), "Current joint velocities: %f, %f, %f, %f, %f, %f, %f, %f",
    //     current_joint_velocities_(0), current_joint_velocities_(1), current_joint_velocities_(2),
    //     current_joint_velocities_(3), current_joint_velocities_(4), current_joint_velocities_(5),
    //     current_joint_velocities_(6), current_joint_velocities_(7));
}

void IKSolverNode::publishJointTrajectoryControllerState()
{
    // Create the message to be published.
    control_msgs::msg::JointTrajectoryControllerState joint_trajectory_controller_state
        = convertToJointTrajectoryControllerStateMsg();

    // Publish the message.
    joint_trajectory_controller_state_pub_->publish(joint_trajectory_controller_state);
}

std::vector<Eigen::VectorXd> IKSolverNode::calculateError()
{
    // Define a vector of Eigen::VectorXd to store the error.
    std::vector<Eigen::VectorXd> error;

    // Calculate error.
    Eigen::VectorXd error_joint_positions_ = desired_joint_positions_ - current_joint_positions_;
    Eigen::VectorXd error_joint_velocities_ = desired_joint_velocities_ - current_joint_velocities_;

    // Push the error to the vector.
    error.push_back(error_joint_positions_);
    error.push_back(error_joint_velocities_);

    return error;
}

control_msgs::msg::JointTrajectoryControllerState IKSolverNode::convertToJointTrajectoryControllerStateMsg()
{
    // Calculate the error.
    std::vector<Eigen::VectorXd> error_joints = calculateError();

    // Convert Eigen::VectorXd to a vector of doubles.
    std::vector<double> desired_positions_vector(
        desired_joint_positions_.data(), desired_joint_positions_.data() + desired_joint_positions_.size());
    std::vector<double> desired_velocities_vector(
        desired_joint_velocities_.data(), desired_joint_velocities_.data() + desired_joint_velocities_.size());
    std::vector<double> current_positions_vector(
        current_joint_positions_.data(), current_joint_positions_.data() + current_joint_positions_.size());
    std::vector<double> current_velocities_vector(
        current_joint_velocities_.data(), current_joint_velocities_.data() + current_joint_velocities_.size());
    std::vector<double> error_positions_vector(error_joints[0].data(), error_joints[0].data() + error_joints[0].size());
    std::vector<double> error_velocities_vector(
        error_joints[1].data(), error_joints[1].data() + error_joints[1].size());

    // Create the message.
    control_msgs::msg::JointTrajectoryControllerState joint_trajectory_controller_state;
    joint_trajectory_controller_state.header.stamp = this->now();
    joint_trajectory_controller_state.joint_names
    = { "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", 
        "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle" };

    // Publish the desired joint positions and velocities.
    joint_trajectory_controller_state.desired.positions = desired_positions_vector;
    joint_trajectory_controller_state.desired.velocities = desired_velocities_vector;

    // Publish the actual joint positions and velocities.
    joint_trajectory_controller_state.actual.positions = current_positions_vector;
    joint_trajectory_controller_state.actual.velocities = current_velocities_vector;

    // Publish the error joint positions and velocities.
    joint_trajectory_controller_state.error.positions = error_positions_vector;
    joint_trajectory_controller_state.error.velocities = error_velocities_vector;

    return joint_trajectory_controller_state;
}

void IKSolverNode::calculateDesiredJointStates()
{
    // Get the current joint positions from the state estimation node.
    current_joint_positions_request_ = std::make_shared<march_shared_msgs::srv::GetCurrentJointPositions::Request>();
    current_joint_positions_future_
        = current_joint_positions_client_->async_send_request(current_joint_positions_request_,
            std::bind(&IKSolverNode::currentJointPositionsCallback, this, std::placeholders::_1));
}

void IKSolverNode::currentJointPositionsCallback(
    const rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture future)
{
    RCLCPP_INFO(this->get_logger(), "Receiving response from /get_current_joint_positions...");
    if (future.get()->status) {
        RCLCPP_INFO(this->get_logger(), "Current joint positions received.");
        current_joint_positions_
            = Eigen::Map<Eigen::VectorXd>(current_joint_positions_future_.get()->joint_positions.data(),
                current_joint_positions_future_.get()->joint_positions.size());

        current_joint_velocities_
            = Eigen::Map<Eigen::VectorXd>(current_joint_positions_future_.get()->joint_velocities.data(),
                current_joint_positions_future_.get()->joint_velocities.size());

        RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f",
            current_joint_positions_(0), current_joint_positions_(1), current_joint_positions_(2),
            current_joint_positions_(3), current_joint_positions_(4), current_joint_positions_(5),
            current_joint_positions_(6), current_joint_positions_(7));

        RCLCPP_INFO(this->get_logger(), "Integrating the joint velocities.");
        desired_joint_positions_ = ik_solver_.integrateJointVelocities();

        // Print the desired joint positions.
        RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", desired_joint_positions_(0),
            desired_joint_positions_(1), desired_joint_positions_(2), desired_joint_positions_(3),
            desired_joint_positions_(4), desired_joint_positions_(5), desired_joint_positions_(6),
            desired_joint_positions_(7));

        // Publish the desired joint positions and velocities.
        RCLCPP_INFO(this->get_logger(), "Publishing the desired joint positions and velocities.");
        publishJointTrajectoryControllerState();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current joint positions.");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSolverNode>());
    // auto node = std::make_shared<IKSolverNode>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();
    rclcpp::shutdown();
    return 0;
}
