//
// Created by rixt on 14-4-23.
//

#include "../include/march_hardware_interface/weight_node.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

WeightNode::WeightNode()
        : Node("weight_node")
{
    m_weight_subscription = this->create_subscription<march_shared_msgs::msg::WeightStamped>(
            "fuzzy_weight", 10, std::bind(&WeightNode::weight_callback, this, _1));

    this->declare_parameter("allowed_source", "fuzzy_node");
}

void WeightNode::weight_callback(march_shared_msgs::msg::WeightStamped::SharedPtr msg)
{

    std::string allowed_source = this->get_parameter("allowed_source").as_string();
    if (msg->header.frame_id != allowed_source) //TODO: put a variable here
    {
        // The message was published by an unexpected node
        // "fuzzy_node" when using a fuzzy control gait, and "gait_node" when using a position or torque specific gait
        RCLCPP_WARN_STREAM(rclcpp::get_logger("weight_node"), "Received message from a blocked source: " << msg->header.frame_id);
        return;
    }

    // get letter of msg
    const char leg = msg->leg;
    if(leg != 'l' and leg != 'r'){
        RCLCPP_WARN(this->get_logger(), "Invalid character provided in weight message: %c! Provide either 'l' or 'r'.", leg);
    }

    // apply to all the joints of that leg
    std::vector<march_hardware_interface::JointInfo>* joints_info_ = m_hardware_interface->getJointsInfo();
    for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
        if(leg == 'l' and jointInfo.name.find("left") != std::string::npos){
            jointInfo.torque_weight = msg->torque_weight;
            jointInfo.position_weight = msg->position_weight;
        }
        else if(leg == 'r' and jointInfo.name.find("right") != std::string::npos){
            jointInfo.torque_weight = msg->torque_weight;
            jointInfo.position_weight = msg->position_weight;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Joint %s seems to be part of neither the right nor the left leg...", jointInfo.name);
        }
    }
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