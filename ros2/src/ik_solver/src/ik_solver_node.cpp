#include "ik_solver/ik_solver_node.hpp"
using std::placeholders::_1;

IKSolverNode::IKSolverNode()
  : Node("ik_solver")
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&IKSolverNode::timerCallback, this));
  RCLCPP_INFO(this->get_logger(), "IKSolverNode has been started.");
  // Eigen::VectorXd joint_config = ik_solver_.getJointConfig();
  // RCLCPP_INFO(this->get_logger(), "Joint configuration: %f, %f, %f, %f", joint_config(0), joint_config(1), joint_config(2), joint_config(3));

  ik_solver_command_sub_ = this->create_subscription<march_shared_msgs::msg::IksFootPositions>(
    "topic", 10, std::bind(&IKSolverNode::IksFootPositionsCallback, this, std::placeholders::_1));

}

void IKSolverNode::IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "IKSolverCommand received.");
  // std::vector<Eigen::VectorXd> desired_poses;
  // // TODO: Apply for loop after MVP.
  // geometry_msgs::msg::Point desired_pose_left = msg->desired_pose_left;
  // geometry_msgs::msg::Point desired_pose_right = msg->desired_pose_right;
  // Eigen::VectorXd desired_pose = Eigen::VectorXd(
  //   desired_pose_left.x, desired_pose_left.y, desired_pose_left.z,
  //   desired_pose_right.x, desired_pose_right.y, desired_pose_right.z);
  // desired_poses.push_back(desired_pose);

  // ik_solver_.updateDesiredPoses(desired_poses);
  // Eigen::VectorXd joint_config = ik_solver_.solve();
  // RCLCPP_INFO(this->get_logger(), 
  //   "Joint configuration: %f, %f, %f, %f, %f, %f, %f, %f", 
  //   joint_config(0), joint_config(1), joint_config(2), joint_config(3),
  //   joint_config(4), joint_config(5), joint_config(6), joint_config(7));
}

void IKSolverNode::timerCallback()
{
  RCLCPP_INFO(this->get_logger(), "IKSolverNode.");
  Eigen::MatrixXd jacobian = *ik_solver_.getJacobianPtr(0);
  for (int i = 0; i < jacobian.rows(); i++) {
    RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f, %f, %f, %f, %f, %f]", 
      jacobian(i, 0), jacobian(i, 1), jacobian(i, 2), jacobian(i, 3), 
      jacobian(i, 4), jacobian(i, 5), jacobian(i, 6), jacobian(i, 7));
  }
  RCLCPP_INFO(this->get_logger(), "----------------------------------------");
  Eigen::MatrixXd jacobian_inverse = *ik_solver_.getJacobianInversePtr(0);
  for (int i = 0; i < jacobian_inverse.rows(); i++) {
    RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f, %f, %f, %f]", 
      jacobian_inverse(i, 0), jacobian_inverse(i, 1), jacobian_inverse(i, 2),
      jacobian_inverse(i, 3), jacobian_inverse(i, 4), jacobian_inverse(i, 5));
  }
  RCLCPP_INFO(this->get_logger(), "----------------------------------------");

  std::vector<Eigen::VectorXd> desired_poses;  
  Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
  desired_pose << 4.0, 5.0, 6.0, 4.0, 5.0, 6.0;
  desired_poses.push_back(desired_pose);
  auto desired_joint_velocities = ik_solver_.solve(desired_poses);
  RCLCPP_INFO(this->get_logger(), "Desired joint velocities: [%f, %f, %f, %f, %f, %f, %f, %f]", 
    desired_joint_velocities(0), desired_joint_velocities(1), desired_joint_velocities(2), desired_joint_velocities(3),
    desired_joint_velocities(4), desired_joint_velocities(5), desired_joint_velocities(6), desired_joint_velocities(7));
}

// void IKSolverNode::getJacobian()
// {
//   Eigen::MatrixXd jacobian = *ik_solver_.getJacobianPtr(0);
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKSolverNode>());
  rclcpp::shutdown();
  return 0;
}
