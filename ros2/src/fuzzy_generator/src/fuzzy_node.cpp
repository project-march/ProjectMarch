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
    // set the stance leg
    m_fuzzy_generator.setStanceLeg(*msg.get());

    // update the weights for the left leg
    Leg* left_leg = m_fuzzy_generator.getLeftLeg();
    m_fuzzy_generator.updateWeights(left_leg);

    // send the weights for the left leg
    march_shared_msgs::msg::WeightStamped left_weights;
    left_weights.torque_weight = left_leg->getTorqueWeight();
    left_weights.position_weight = left_leg->getPositionWeight();
    left_weights.leg = 'l';
    m_publish_weight->publish(left_weights);

    // update the weights for the right leg
    Leg* right_leg = m_fuzzy_generator.getRightLeg();
    m_fuzzy_generator.updateWeights(right_leg);

    // send the weights for the left leg
    march_shared_msgs::msg::WeightStamped right_weights;
    right_weights.torque_weight = right_leg->getTorqueWeight();
    right_weights.position_weight = right_leg->getPositionWeight();
    right_weights.leg = 'r';
    m_publish_weight->publish(right_weights);
}

void FuzzyNode::height_callback(march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg){

    // update the feet height
    m_fuzzy_generator.setFeetHeight(*msg.get());

    // update the weights for the left leg
    Leg* left_leg = m_fuzzy_generator.getLeftLeg();
    m_fuzzy_generator.updateWeights(left_leg);

    // send the weights for the left leg
    march_shared_msgs::msg::WeightStamped left_weights;
    left_weights.torque_weight = left_leg->getTorqueWeight();
    left_weights.position_weight = left_leg->getPositionWeight();
    left_weights.leg = 'l';
    left_weights.header.frame_id = this->get_name();
    m_publish_weight->publish(left_weights);

    // update the weights for the right leg
    Leg* right_leg = m_fuzzy_generator.getRightLeg();
    m_fuzzy_generator.updateWeights(right_leg);

    // send the weights for the left leg
    march_shared_msgs::msg::WeightStamped right_weights;
    right_weights.torque_weight = right_leg->getTorqueWeight();
    right_weights.position_weight = right_leg->getPositionWeight();
    right_weights.leg = 'r';
    right_weights.header.frame_id = this->get_name();
    m_publish_weight->publish(right_weights);
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