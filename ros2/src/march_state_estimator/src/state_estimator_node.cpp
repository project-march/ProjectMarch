/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description.hpp"
#include "march_state_estimator/robot_description_node.hpp"
#include "march_state_estimator/sensor_fusion_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include "omp.h"
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    Eigen::initParallel();
    rclcpp::init(argc, argv);

    std::string yaml_filename = "robot_definition-hennie_with_koen.yaml";
    RobotDescription::SharedPtr robot_description = std::make_shared<RobotDescription>(yaml_filename);

    rclcpp::Node::SharedPtr node_robot_description = std::make_shared<RobotDescriptionNode>(robot_description);
    rclcpp::Node::SharedPtr node_sensor_fusion = std::make_shared<SensorFusionNode>(robot_description);

    // TODO: Fix multi-threaded executor due to issue in service handling in RobotDescriptionNode.
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_robot_description);
    executor.add_node(node_sensor_fusion);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}