//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_node.hpp"
#include "fuzzy_generator/fuzzy_generator.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

FuzzyNode::FuzzyNode()
        : Node("fuzzy_node")
        , m_fuzzy_generator()
{
    m_position_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "robot_feet_positions", 10, std::bind(&FuzzyNode::position_callback, this, _1));
    m_torque_subscription = this->create_subscription<std_msgs::msg::Float32>( //TODO: apply correct message type
            "robot_feet_torque", 10, std::bind(&FuzzyNode::torque_callback, this, _1));
    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FeetHeightStamped>(
            "robot_feet_height", 10, std::bind(&FuzzyNode::height_callback, this, _1));
    m_stance_leg_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "current_stance_foot", 10, std::bind(&FuzzyNode::stance_leg_callback, this, _1));
    m_publish_pos_weight = this->create_publisher<std_msgs::msg::Float32>("position_weight", 10); //TODO: connect to HWI
    m_publish_torque_weight = this->create_publisher<std_msgs::msg::Float32>("torque_weight", 10); //TODO: connect to HWI
}

void FuzzyNode::position_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    Leg* stance_leg = m_fuzzy_generator.getStanceLeg();
//    m_fuzzy_generator.swing_leg->dummyFunction();
    // the position being published is always that of the stance leg
    stance_leg->setPosition(msg->point);
    m_fuzzy_generator.updateWeights(stance_leg);
    m_publish_pos_weight->publish(stance_leg->getPositionWeight());
    m_publish_torque_weight->publish(stance_leg->getTorqueWeight());
}

void FuzzyNode::torque_callback(std_msgs::msg::Float32::SharedPtr msg)
{
    Leg* stance_leg = m_fuzzy_generator.getStanceLeg();
    //TODO: there is no publisher on this topic yet so we don't know which leg it will update
    stance_leg->setTorque(*msg.get());
    m_fuzzy_generator.updateWeights(stance_leg);
    m_publish_pos_weight->publish(stance_leg->getPositionWeight());
    m_publish_torque_weight->publish(stance_leg->getTorqueWeight());
}

void FuzzyNode::stance_leg_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_fuzzy_generator.setStanceLeg(*msg.get());
}

void FuzzyNode::height_callback(march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg){
    m_fuzzy_generator.setFeetHeight(*msg.get());
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
    rclcpp::spin(std::make_shared<FuzzyNode>());

    rclcpp::shutdown();
    return 0;
}