//
// Created by rixt on 14-4-23.
//

#include "../include/march_hardware_interface/weight_node.h"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

WeightNode::WeightNode()
        : Node("weight_node")
{
    m_weight_subscription = this->create_subscription<march_shared_msgs::msg::WeightStamped>(
            "fuzzy_weight", 10, std::bind(&WeightNode::weight_callback, this, _1));
}

void WeightNode::weight_callback(march_shared_msgs::msg::WeightStamped::SharedPtr msg)
{
    std::vector<march_hardware_interface::JointInfo>* joints_info_ = m_hardware_interface->getJointsInfo();
    //TODO: implement
    // get all the joints from either the left or the right, and apply the torque and position weights to the JointInfo
}

/**
 * Main function to run the node.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightNode>());

    rclcpp::shutdown();
    return 0;
}