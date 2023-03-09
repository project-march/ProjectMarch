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