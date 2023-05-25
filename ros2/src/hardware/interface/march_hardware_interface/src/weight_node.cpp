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
//    m_fuzzy_weight_subscription = this->create_subscription<march_shared_msgs::msg::WeightStamped>(
//            "fuzzy_weight", 10, std::bind(&WeightNode::fuzzy_weight_callback, this, _1));

//    m_control_type_subscription = this->create_subscription<std_msgs::msg::String>(
//            "/march/weight_control_type", 10, std::bind(&WeightNode::control_type_callback, this, _1));
//
//    m_direct_torque_subscription = this->create_subscription<std_msgs::msg::Float32>(
//            "/march/direct_torque", 10, std::bind(&WeightNode::direct_torque_callback, this, _1));
//
//    this->declare_parameter("allowed_control_type", "fuzzy");
}

///**
// * Sets the allowed type of weights for torque and position
// *
// * @param msg Message that contains the control type: Position, Torque, or Fuzzy
// * @return
// */
//void WeightNode::control_type_callback(std_msgs::msg::String::SharedPtr msg) {
//    std::string allowed_control_type = msg->data;
//    RCLCPP_INFO_STREAM(this->get_logger(), "setting weights according to " << allowed_control_type << " control ");
//
//    this->set_parameter(rclcpp::Parameter("allowed_control_type", allowed_control_type));
//    if(allowed_control_type == "position"){
//        setJointsWeight("l", 1, 0);
//        setJointsWeight("r", 1, 0);
//    }
//    if(allowed_control_type == "torque"){
//        setJointsWeight("l", 0, 1);
//        setJointsWeight("r", 0, 1);
//    }
//}

///**
// * Processes the weights sent from the fuzzy generator
// *
// * @param msg Message that contains the weights for both torque and position
// * @return
// */
//void WeightNode::fuzzy_weight_callback(march_shared_msgs::msg::WeightStamped::SharedPtr msg)
//{
//    std::string allowed_control_type = this->get_parameter("allowed_control_type").as_string();
//    if (msg->header.frame_id.find(allowed_control_type) == std::string::npos)
//    {
//        // The message was published by a blocked source
//        RCLCPP_INFO_STREAM(rclcpp::get_logger("weight_node"), "Ignoring weight message from a blocked control type: " << msg->header.frame_id << ". Allowed control type is " <<allowed_control_type);
//        return;
//    }
//
//    setJointsWeight(msg->leg, msg->position_weight, msg->torque_weight);
//}

///**
// * Processes the weights sent from the fuzzy generator
// *
// * @param msg Message that contains the weights for both torque and position
// * @return
// */
//void WeightNode::direct_torque_callback(std_msgs::msg::Float32::SharedPtr msg)
//{
//    std::string allowed_control_type = this->get_parameter("allowed_control_type").as_string();
//    if (allowed_control_type.find("torque") == std::string::npos)
//    {
//        // Directly calling torque should not be possible in any other mode than torque control
//        RCLCPP_INFO_STREAM(rclcpp::get_logger("weight_node"), "We are not in torque control, so we will ignore the direct call. We are in control mode: " <<allowed_control_type);
//        return;
//    }
//
//    float torque = msg->data;
//
//    // We are setting the torque immediately
//    RCLCPP_INFO(rclcpp::get_logger("weight_node"), "We are setting the torque directly to: %f", torque);
//
//    // apply to all the joints of that leg
//    std::vector<march_hardware_interface::JointInfo>* joints_info_ = m_hardware_interface->getJointsInfo();
//
//    for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
//        jointInfo.torque = torque;
//    }
//}

///**
// * Applies torque and position weights to all JointInfo objects in the hardware interface
// *
// * @param leg Either "l" or "r" to indicate left or right leg
// * @param position_weight A float between 0 and 1 to apply to joints
// * @param torque_weight A float between 0 and 1 to apply to joints
// * @return
// */
//void WeightNode::setJointsWeight(std::string leg, float position_weight, float torque_weight){
//
//    // check the leg choice
//    if(leg != "l" and leg != "r"){
//        RCLCPP_WARN(this->get_logger(), "Invalid character provided in weight message: %c! Provide either 'l' or 'r'.", leg);
//        return;
//    }
//    RCLCPP_INFO(this->get_logger(), "Setting weights of %s leg", leg);
//    if(m_hardware_interface == NULL ){
//        RCLCPP_INFO(this->get_logger(), "No hwi initialized");
//    }
//
//
//    // apply to all the joints of that leg
//    std::vector<march_hardware_interface::JointInfo>* joints_info_ = m_hardware_interface->getJointsInfo();
//
//    for (march_hardware_interface::JointInfo& jointInfo : *joints_info_) {
//        if(leg == "l" and jointInfo.name.find("left") != std::string::npos){
//            jointInfo.torque_weight = torque_weight;
//            jointInfo.position_weight = position_weight;
//        }
//        else if(leg == "r" and jointInfo.name.find("right") != std::string::npos){
//            jointInfo.torque_weight = torque_weight;
//            jointInfo.position_weight = position_weight;
//        }
//        else{
//            RCLCPP_WARN(this->get_logger(), "Joint %s seems to be part of neither the right nor the left leg...", jointInfo.name);
//        }
//    }
//}

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