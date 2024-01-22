#include "march_state_estimator/sensor_fusion_node.hpp"
#include "march_state_estimator/robot_description_node.hpp"
#include "march_state_estimator/robot_description.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <vector>
#include <memory>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // declare_parameter<std::string>("urdf_path", ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf");
    // std::string urdf_path = get_parameter("urdf_path").as_string();
    std::string urdf_path = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf";

    std::shared_ptr<RobotDescription> robot_description = std::make_shared<RobotDescription>();
    robot_description->parseURDF(urdf_path);
    robot_description->configureRobotNodes();

    auto sensor_fusion_node = std::make_shared<SensorFusionNode>(robot_description);
    auto robot_description_node = std::make_shared<RobotDescriptionNode>(robot_description); 

    // Multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensor_fusion_node);
    executor.add_node(robot_description_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}