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
    m_publish_curve = this->create_publisher<geometry_msgs::msg::PointStamped>("bezier_trajectory", 10);
    m_points_subscription = this->create_subscription<march_shared_msgs::msg::PointStampedList>(
        "bezier_points", 10, std::bind(&SwingLegTrajectoryGeneratorNode::subscriber_callback, this, _1));

    m_swing_leg_generator = SwingLegTrajectoryGenerator();
}

void SwingLegTrajectoryGeneratorNode::subscriber_callback(march_shared_msgs::msg::PointStampedList::SharedPtr msg)
{
    m_swing_leg_generator.setPoints(msg->points);
    RCLCPP_INFO((this->get_logger()), "points updated with x = %f", msg->points.at(1).point.x);
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