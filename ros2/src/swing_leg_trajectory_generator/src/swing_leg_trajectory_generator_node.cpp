//
// Created by Marco Bak march8 on 1-2-23.
//
#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator_node.hpp"
#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

SwingLegTrajectoryGeneratorNode::SwingLegTrajectoryGeneratorNode()
    : Node("swing_leg_trajectory_generator_node")
{
    //    m_publish_curve = this->create_publisher<geometry_msgs::msg::PointStamped>("bezier_points", 10);
    m_publish_curve = this->create_publisher<geometry_msgs::msg::PoseArray>("bezier_trajectory", 10);
    m_points_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "bezier_points", 10, std::bind(&SwingLegTrajectoryGeneratorNode::subscriber_callback, this, _1));
    m_final_feet_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>("final_feet_position", 10,
    std::bind(&SwingLegTrajectoryGeneratorNode::final_feet_callback, this, _1));
    m_swing_leg_generator = SwingLegTrajectoryGenerator();
}

void SwingLegTrajectoryGeneratorNode::subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    //TODO: implement
}

void SwingLegTrajectoryGeneratorNode::final_feet_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    //TODO: update implementation
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
    rclcpp::spin(std::make_shared<SwingLegTrajectoryGeneratorNode>());

    rclcpp::shutdown();
    return 0;
}