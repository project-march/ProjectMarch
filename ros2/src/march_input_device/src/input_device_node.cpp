//
// Created by andrew on 23-11-23.
//

#include "march_input_device/input_device_node.hpp"
#include "march_input_device/input_device.hpp"

inputDeviceNode::inputDeviceNode()
    : Node("march_input_device_node"),
    m_ipd (IPD())
{
    m_new_state_publisher = create_publisher<std_msgs::msg::Int32>("new_state", 10);
}

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  // Spin the node to start processing messages
  rclcpp::spin(std::make_shared<inputDeviceNode>());

  rclcpp::shutdown();
  return 0;
}