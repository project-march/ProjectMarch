#include "march_state_estimator/march_state_estimator_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

MarchStateEstimatorNode::MarchStateEstimatorNode()
: Node("march_state_estimator_node")
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode constructor");
    robot_description_ = std::make_shared<RobotDescription>();
    robot_description_->parseURDF(ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf");
    robot_description_->configureRobotNodes();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchStateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}