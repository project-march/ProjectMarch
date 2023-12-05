#include "march_ik_solver/ik_solver_node.hpp"
using std::placeholders::_1;

IKSolverNode::IKSolverNode()
  : Node("ik_solver")
{
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&IKSolverNode::timerCallback, this));
  ik_solver_command_sub_ = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
    "iks_foot_positions", 10, std::bind(&IKSolverNode::IksFootPositionsCallback, this, std::placeholders::_1));
  robot_state_sub_ = this->create_subscription<march_shared_msgs::msg::RobotState>(
    "current_robot_state", 10, std::bind(&IKSolverNode::RobotStateCallback, this, std::placeholders::_1));
  robot_state_pub_ = this->create_publisher<march_shared_msgs::msg::RobotState>("desired_robot_state", 1); // TODO: Change queue.
  current_joint_positions_client_ = this->create_client<march_shared_msgs::srv::GetCurrentJointPositions>("get_current_joint_positions");

  // Configure IK solver.
  std::vector <uint8_t> tasks_m = ik_solver_.getTasksM();
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
  geometry_msgs::msg::Point desired_pose_left = msg->left_foot_position[0];
  geometry_msgs::msg::Point desired_pose_right = msg->right_foot_position[0];
  Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
  desired_pose << desired_pose_left.x, desired_pose_left.y, desired_pose_left.z,
                  desired_pose_right.x, desired_pose_right.y, desired_pose_right.z;
  desired_poses.push_back(desired_pose);

  // Solve the IK problem to obtain the desired joint velocities.
  RCLCPP_INFO(this->get_logger(), "Solving the IK problem...");
  desired_joint_velocities_ = ik_solver_.solve(desired_poses);

  // Print the desired joint velocities.
 RCLCPP_INFO(this->get_logger(),
    "Joint velocities: %f, %f, %f, %f, %f, %f, %f, %f", 
    desired_joint_velocities_(0), desired_joint_velocities_(1), desired_joint_velocities_(2), desired_joint_velocities_(3),
    desired_joint_velocities_(4), desired_joint_velocities_(5), desired_joint_velocities_(6), desired_joint_velocities_(7));

  // Integrate the joint velocities to obtain the desired joint positions.
  RCLCPP_INFO(this->get_logger(), "Getting the current joint positions...");
  calculateDesiredJointStates();
}

void IKSolverNode::RobotStateCallback(const march_shared_msgs::msg::RobotState::SharedPtr msg)
{
  // Update the current joint positions.
  Eigen::VectorXd current_joint_positions = Eigen::Map<Eigen::VectorXd>(msg->joint_pos.data(), msg->joint_pos.size());
  current_joint_positions_ = current_joint_positions;

  // Print the current joint positions.
  RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f", 
    current_joint_positions_(0), current_joint_positions_(1), current_joint_positions_(2), current_joint_positions_(3),
    current_joint_positions_(4), current_joint_positions_(5), current_joint_positions_(6), current_joint_positions_(7));
}

void IKSolverNode::publishJointState(const Eigen::VectorXd desired_joint_positions, const Eigen::VectorXd desired_joint_velocities)
{
  // Convert Eigen::VectorXd to a vector of doubles.
  std::vector<double> positions_vector(desired_joint_positions.data(), desired_joint_positions.data() + desired_joint_positions.size());
  std::vector<double> velocities_vector(desired_joint_velocities.data(), desired_joint_velocities.data() + desired_joint_velocities.size());

  // Publish the desired joint positions and velocities.
  march_shared_msgs::msg::RobotState robot_state;
  robot_state.joint_pos = positions_vector;
  robot_state.joint_vel = velocities_vector;
  robot_state_pub_->publish(robot_state);
}

  void IKSolverNode::calculateDesiredJointStates()
  {
  //Get the current joint positions from the state estimation node.
  current_joint_positions_request_ = std::make_shared<march_shared_msgs::srv::GetCurrentJointPositions::Request>();
  current_joint_positions_future_ = current_joint_positions_client_->async_send_request(
    current_joint_positions_request_, 
    std::bind(&IKSolverNode::currentJointPositionsCallback, this, std::placeholders::_1));
}

void IKSolverNode::currentJointPositionsCallback(
  const rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedFuture future)
{
  RCLCPP_INFO(this->get_logger(), "Receiving response from /get_current_joint_positions...");
  if (future.get()->status)
  {
    RCLCPP_INFO(this->get_logger(), "Current joint positions received.");
    current_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
      current_joint_positions_future_.get()->joint_positions.data(), 
      current_joint_positions_future_.get()->joint_positions.size());

    RCLCPP_INFO(this->get_logger(), "Current joint positions: %f, %f, %f, %f, %f, %f, %f, %f", 
      current_joint_positions_(0), current_joint_positions_(1), current_joint_positions_(2), current_joint_positions_(3),
      current_joint_positions_(4), current_joint_positions_(5), current_joint_positions_(6), current_joint_positions_(7));

    RCLCPP_INFO(this->get_logger(), "Integrating the joint velocities.");
    desired_joint_positions_ = ik_solver_.integrateJointVelocities();

// Print the desired joint positions.
    RCLCPP_INFO(this->get_logger(), 
      "Joint positions: %f, %f, %f, %f, %f, %f, %f, %f", 
      desired_joint_positions_(0), desired_joint_positions_(1), desired_joint_positions_(2), desired_joint_positions_(3),
      desired_joint_positions_(4), desired_joint_positions_(5), desired_joint_positions_(6), desired_joint_positions_(7));

    // Publish the desired joint positions and velocities.
    RCLCPP_INFO(this->get_logger(), "Publishing the desired joint positions and velocities.");
    publishJointState(desired_joint_positions_, desired_joint_velocities_);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current joint positions.");
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
  // auto node = std::make_shared<IKSolverNode>();
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(node);
  // executor.spin();
  rclcpp::shutdown();
  return 0;
}
