//
// Created by rixt on 16-5-23.
//

#include "joint_trajectory_buffer/joint_trajectory_buffer.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryBufferNode>());
    rclcpp::shutdown();
    return 0;
}
