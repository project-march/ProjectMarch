#include "march_ik_solver/ik_solver_node.hpp"
using std::placeholders::_1;

IKSolverNode::IKSolverNode()
  : Node("ik_solver")
{
  // Configure IK solver.
  std::vector <uint8_t> tasks_m = ik_solver_.getTasksM();
  for (auto task_m : tasks_m) {
    Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(task_m);
    desired_poses_.push_back(desired_pose);
  }
  ik_solver_.setCurrentJointPositionsPtr(&current_joint_positions_);

  // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&IKSolverNode::timerCallback, this));
  ik_solver_command_sub_ = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
    "iks_foot_positions", 10, std::bind(&IKSolverNode::IksFootPositionsCallback, this, std::placeholders::_1));
  // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  current_joint_positions_client_ = this->create_client<march_shared_msgs::srv::GetCurrentJointPositions>("get_current_joint_positions");

  RCLCPP_INFO(this->get_logger(), "IKSolverNode has been started.");
}

void IKSolverNode::IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "IKSolverCommand received.");
  std::vector<Eigen::VectorXd> desired_poses;
  // TODO: Apply for loop after MVP.
  geometry_msgs::msg::Point desired_pose_left = msg->left_foot_position[0];
  geometry_msgs::msg::Point desired_pose_right = msg->right_foot_position[0];
  Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
  desired_pose << desired_pose_left.x, desired_pose_left.y, desired_pose_left.z,
                  desired_pose_right.x, desired_pose_right.y, desired_pose_right.z;
  desired_poses.push_back(desired_pose);

  Eigen::VectorXd joint_config = ik_solver_.solve(desired_poses);
  RCLCPP_INFO(this->get_logger(), 
    "Joint configuration: %f, %f, %f, %f, %f, %f, %f, %f", 
    joint_config(0), joint_config(1), joint_config(2), joint_config(3),
    joint_config(4), joint_config(5), joint_config(6), joint_config(7));
}

Eigen::VectorXd IKSolverNode::getCurrentJointPositions()
{
  // Get the current joint positions from the state estimation node.
  auto request = std::make_shared<march_shared_msgs::srv::GetCurrentJointPositions::Request>();
  auto result = current_joint_positions_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) 
  {
    Eigen::VectorXd current_joint_positions = Eigen::Map<Eigen::VectorXd>(result.get()->joint_positions.data(), 8); // TODO: Change 8 to n_joints_.

    // Update the current joint positions.
    current_joint_positions_ = current_joint_positions;

    return current_joint_positions;

  } else {
    // TODO: Add error handling.
    RCLCPP_ERROR(this->get_logger(), "Failed to get current joint positions. Returning the previous joint positions.");
    return current_joint_positions_;
  }
}

// void IKSolverNode::timerCallback()
// {
//   // A test function that requests the task server per time step.
//   RCLCPP_INFO(this->get_logger(), "IKSolverNode.");
//   Eigen::MatrixXd jacobian = *ik_solver_.getJacobianPtr(0);
//   for (int i = 0; i < jacobian.rows(); i++) {
//     RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f, %f, %f, %f, %f, %f]", 
//       jacobian(i, 0), jacobian(i, 1), jacobian(i, 2), jacobian(i, 3), 
//       jacobian(i, 4), jacobian(i, 5), jacobian(i, 6), jacobian(i, 7));
//   }
//   RCLCPP_INFO(this->get_logger(), "----------------------------------------");
//   Eigen::MatrixXd jacobian_inverse = *ik_solver_.getJacobianInversePtr(0);
//   for (int i = 0; i < jacobian_inverse.rows(); i++) {
//     RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f, %f, %f, %f]", 
//       jacobian_inverse(i, 0), jacobian_inverse(i, 1), jacobian_inverse(i, 2),
//       jacobian_inverse(i, 3), jacobian_inverse(i, 4), jacobian_inverse(i, 5));
//   }
//   RCLCPP_INFO(this->get_logger(), "----------------------------------------");

//   std::vector<Eigen::VectorXd> desired_poses;  
//   Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
//   desired_pose << 4.0, 5.0, 6.0, 4.0, 5.0, 6.0;
//   desired_poses.push_back(desired_pose);
//   auto desired_joint_velocities = ik_solver_.solve(desired_poses);
//   RCLCPP_INFO(this->get_logger(), "Desired joint velocities: [%f, %f, %f, %f, %f, %f, %f, %f]", 
//     desired_joint_velocities(0), desired_joint_velocities(1), desired_joint_velocities(2), desired_joint_velocities(3),
//     desired_joint_velocities(4), desired_joint_velocities(5), desired_joint_velocities(6), desired_joint_velocities(7));
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKSolverNode>());
  rclcpp::shutdown();
  return 0;
}
