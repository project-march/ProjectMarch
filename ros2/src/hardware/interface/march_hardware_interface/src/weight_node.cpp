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
    //TODO: implement

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