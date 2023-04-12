//
// Created by rixt on 12-4-23.
//

#include "fuzzy_decider/fuzzy_node.hpp"
#include "fuzzy_decider/fuzzy_decider.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

FuzzyNode::FuzzyNode()
    : Node("fuzzy_node")
    , m_fuzzy_decider()
{
    m_position_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "position", 10, std::bind(&FuzzyNode::position_callback, this, _1));
    m_torque_subscription = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "torque", 10, std::bind(&FuzzyNode::torque_callback, this, _1));
    m_publish_pos_weight = this->create_publisher<geometry_msgs::msg::PointStamped>("position_weight", 10);
    m_publish_torque_weight = this->create_publisher<geometry_msgs::msg::PointStamped>("torque_weight", 10);
}

void FuzzyNode::position_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_fuzzy_decider.setPosition(msg->point);
}

void FuzzyNode::torque_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_fuzzy_decider.setTorque(msg->point);
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