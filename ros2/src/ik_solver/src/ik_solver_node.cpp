#include "ik_solver/ik_solver_node.hpp"

IKSolverNode::IKSolverNode()
  : Node("ik_solver")
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&IKSolverNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "IKSolverNode has been started.");
  Eigen::VectorXf joint_config = ik_solver_.getJointConfig();
  RCLCPP_INFO(this->get_logger(), "Joint configuration: %f, %f, %f, %f", joint_config(0), joint_config(1), joint_config(2), joint_config(3));
}

void IKSolverNode::timer_callback()
{
  Eigen::MatrixXf jacobian = ik_solver_.getJacobian();
  Eigen::MatrixXf jacobian_inverse = ik_solver_.getJacobianInverse();
  RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f, %f]", jacobian(0, 0), jacobian(0, 1), jacobian(0, 2), jacobian(0, 3));
  RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f, %f]", jacobian(1, 0), jacobian(1, 1), jacobian(1, 2), jacobian(1, 3));
  RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f, %f]", jacobian(2, 0), jacobian(2, 1), jacobian(2, 2), jacobian(2, 3));
  RCLCPP_INFO(this->get_logger(), "----------------------------------------");
  RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f]", jacobian_inverse(0, 0), jacobian_inverse(0, 1), jacobian_inverse(0, 2));
  RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f]", jacobian_inverse(1, 0), jacobian_inverse(1, 1), jacobian_inverse(1, 2));
  RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f]", jacobian_inverse(2, 0), jacobian_inverse(2, 1), jacobian_inverse(2, 2));
  RCLCPP_INFO(this->get_logger(), "[ %f, %f, %f]", jacobian_inverse(3, 0), jacobian_inverse(3, 1), jacobian_inverse(3, 2));
  RCLCPP_INFO(this->get_logger(), "----------------------------------------");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKSolverNode>());
  rclcpp::shutdown();
  return 0;
}
