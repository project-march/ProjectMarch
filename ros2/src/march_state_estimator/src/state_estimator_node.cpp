/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description.hpp"
#include "march_state_estimator/robot_description_node.hpp"
#include "march_state_estimator/sensor_fusion_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // TODO: Fix parameter passing.
    // declare_parameter<std::string>("robot_definition",
    // ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf");
    // std::string yaml_filename = get_parameter("robot_definition").as_string();

    std::string yaml_filename = "robot_definition-hennie_with_koen.yaml";
    std::shared_ptr<RobotDescription> robot_description = std::make_shared<RobotDescription>();
    robot_description->parseYAML("robot_definition-hennie_with_koen.yaml");

    auto sensor_fusion_node = std::make_shared<SensorFusionNode>(robot_description);
    auto robot_description_node = std::make_shared<RobotDescriptionNode>(robot_description);

    // TODO: Fix multi-threaded executor due to issue in service handling in RobotDescriptionNode.
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sensor_fusion_node);
    executor.add_node(robot_description_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}