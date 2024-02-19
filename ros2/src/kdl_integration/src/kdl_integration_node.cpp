#include "kdl_integration/kdl_integration_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <iostream>

KDLIntegrationNode::KDLIntegrationNode()
: Node("kdl_integration_node")
{
    RCLCPP_INFO(this->get_logger(), "Hello from KDLIntegrationNode");

    // Load the robot model
    std::string urdf_model = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf";
    std::cout << "URDF: " << urdf_model << std::endl;
    if (!kdl_parser::treeFromFile(urdf_model, kdl_tree_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl tree");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully constructed kdl tree");
    }

    // Get the chain from the tree
    if (!kdl_tree_.getChain("backpack", "left_ankle", kdl_chain_left_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl chain for left leg");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully constructed kdl chain for left leg");
    }

    if (!kdl_tree_.getChain("backpack", "right_ankle", kdl_chain_right_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl chain for right leg");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully constructed kdl chain for right leg");
    }

    // Get Jacobian for left leg
    KDL::ChainJntToJacSolver jac_solver_left_(kdl_chain_left_);  // (kdl_chain_left_ is a private member of KDLIntegrationNode
    KDL::Jacobian jac_left_;
    KDL::JntArray q_left_;

    jac_left_.resize(kdl_chain_left_.getNrOfJoints());
    q_left_.resize(kdl_chain_left_.getNrOfJoints());

    jac_solver_left_.JntToJac(q_left_, jac_left_);

    // Convert Jacobian to Eigen
    Eigen::MatrixXd jac_left_eigen = Eigen::MatrixXd::Zero(6, kdl_chain_left_.getNrOfJoints());
    for (unsigned int i = 0; i < jac_left_.columns(); i++)
    {
        for (unsigned int j = 0; j < jac_left_.rows(); j++)
        {
            jac_left_eigen(j, i) = jac_left_(j, i);
        }
    }
    std::cout << "Jacobian for left leg: " << std::endl << jac_left_eigen << std::endl;

    // Get Jacobian for right leg
    KDL::ChainJntToJacSolver jac_solver_right_(kdl_chain_right_);
    KDL::Jacobian jac_right_;
    KDL::JntArray q_right_;

    jac_right_.resize(kdl_chain_right_.getNrOfJoints());
    q_right_.resize(kdl_chain_right_.getNrOfJoints());

    jac_solver_right_.JntToJac(q_right_, jac_right_);

    // Convert Jacobian to Eigen
    Eigen::MatrixXd jac_right_eigen = Eigen::MatrixXd::Zero(6, kdl_chain_right_.getNrOfJoints());
    for (unsigned int i = 0; i < jac_right_.columns(); i++)
    {
        for (unsigned int j = 0; j < jac_right_.rows(); j++)
        {
            jac_right_eigen(j, i) = jac_right_(j, i);
        }
    }
    std::cout << "Jacobian for right leg: " << std::endl << jac_right_eigen << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KDLIntegrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}