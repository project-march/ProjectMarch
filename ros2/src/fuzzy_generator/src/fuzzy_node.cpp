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
    m_torque_subscription = this->create_subscription<march_shared_msgs::msg::TorqueStamped>( //TODO: apply correct message type
            "robot_feet_torque", 10, std::bind(&FuzzyNode::torque_callback, this, _1));
    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FeetHeightStamped>(
            "robot_feet_height", 10, std::bind(&FuzzyNode::height_callback, this, _1));
    m_stance_leg_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "current_stance_foot", 10, std::bind(&FuzzyNode::stance_leg_callback, this, _1));
    m_publish_weight = this->create_publisher<march_shared_msgs::msg::WeightStamped>("fuzzy_weight", 10); //TODO: connect to HWI
}

void FuzzyNode::position_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    Leg* stance_leg = m_fuzzy_generator.getStanceLeg();
    // the position being published is always that of the stance leg
    stance_leg->setPosition(msg->point);
    m_fuzzy_generator.updateWeights(stance_leg);
    march_shared_msgs::msg::WeightStamped fuzzy_weights;
    fuzzy_weights.torque_weight = stance_leg->getTorqueWeight();
    fuzzy_weights.position_weight = stance_leg->getPositionWeight();
    fuzzy_weights.leg = stance_leg->side == Left ? 'l' : 'r';
    m_publish_weight->publish(fuzzy_weights);
}

void FuzzyNode::torque_callback(march_shared_msgs::msg::TorqueStamped::SharedPtr msg)
{
    Leg* stance_leg = m_fuzzy_generator.getStanceLeg();
    //TODO: there is no publisher on this topic yet so we don't know which leg it will update whenever torque is passed
    stance_leg->setTorque(*msg.get());
    m_fuzzy_generator.updateWeights(stance_leg);
    march_shared_msgs::msg::WeightStamped fuzzy_weights;
    fuzzy_weights.torque_weight = stance_leg->getTorqueWeight();
    fuzzy_weights.position_weight = stance_leg->getPositionWeight();
    fuzzy_weights.leg = stance_leg->side == Left ? 'l' : 'r';
    m_publish_weight->publish(fuzzy_weights);
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