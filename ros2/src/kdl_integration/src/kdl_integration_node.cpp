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
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully constructed kdl tree");
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KDLIntegrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}