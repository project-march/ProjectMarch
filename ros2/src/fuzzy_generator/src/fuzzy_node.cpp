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
    m_torque_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "robot_feet_torque", 10, std::bind(&FuzzyNode::torque_callback, this, _1));
//    m_foot_height_subscription = this->create_subscription<march_shared_msgs::msg::FeetHeightStamped>(
//            "robot_feet_height", 10, std::bind(&FuzzyNode::height_callback, this, _1));
    m_swing_leg_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "current_stance_foot", 10, std::bind(&FuzzyNode::swing_leg_callback, this, _1));
    m_publish_pos_weight = this->create_publisher<geometry_msgs::msg::PointStamped>("position_weight", 10); //TODO: connect to HWI
    m_publish_torque_weight = this->create_publisher<geometry_msgs::msg::PointStamped>("torque_weight", 10); //TODO: connect to HWI
}

void FuzzyNode::position_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    //TODO: decide on which leg to update, how do we know which leg this was called from? is it always the swing leg?
//    m_fuzzy_generator.setPosition(msg->point);
    //TODO: decide on when to trigger a weight recalculation
}

void FuzzyNode::torque_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    //TODO: decide on which leg to update
//    m_fuzzy_generator.setTorque(msg->point);
    //TODO: decide on when to trigger a weight recalculation
}

void FuzzyNode::swing_leg_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_fuzzy_generator.setStanceLeg(*msg.get());
}

//void FuzzyNode::height_callback(march_shared_msgs::msg::FeetHeightStamped msg){
//
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
    rclcpp::spin(std::make_shared<FuzzyNode>());

    rclcpp::shutdown();
    return 0;
}