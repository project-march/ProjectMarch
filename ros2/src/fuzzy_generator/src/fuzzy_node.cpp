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
    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FeetHeightStamped>(
            "robot_feet_height", 10, std::bind(&FuzzyNode::height_callback, this, _1));
    m_stance_leg_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "current_stance_foot", 10, std::bind(&FuzzyNode::stance_leg_callback, this, _1));
    m_publish_weight = this->create_publisher<march_shared_msgs::msg::WeightStamped>("fuzzy_weight", 10); //TODO: connect to HWI
}

void FuzzyNode::stance_leg_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_fuzzy_generator.setStanceLeg(*msg.get());
    Leg* swing_leg = m_fuzzy_generator.getSwingLeg();
    m_fuzzy_generator.updateWeights(swing_leg);

    march_shared_msgs::msg::WeightStamped fuzzy_weights;
    fuzzy_weights.torque_weight = swing_leg->getTorqueWeight();
    fuzzy_weights.position_weight = swing_leg->getPositionWeight();
    fuzzy_weights.leg = m_fuzzy_generator.getSwingSide();
    m_publish_weight->publish(fuzzy_weights);
}

void FuzzyNode::height_callback(march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg){
    m_fuzzy_generator.setFeetHeight(*msg.get());
    Leg* swing_leg = m_fuzzy_generator.getSwingLeg();
    m_fuzzy_generator.updateWeights(swing_leg);

    march_shared_msgs::msg::WeightStamped fuzzy_weights;
    fuzzy_weights.torque_weight = swing_leg->getTorqueWeight();
    fuzzy_weights.position_weight = swing_leg->getPositionWeight();
    fuzzy_weights.leg = m_fuzzy_generator.getSwingSide();
    m_publish_weight->publish(fuzzy_weights);
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