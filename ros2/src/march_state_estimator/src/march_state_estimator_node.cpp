#include "march_state_estimator/march_state_estimator_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <functional>
#include <chrono>

MarchStateEstimatorNode::MarchStateEstimatorNode()
: Node("march_state_estimator_node")
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode constructor");
    robot_description_ = std::make_shared<RobotDescription>();
    robot_description_->parseURDF(ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf");
    robot_description_->configureRobotNodes();

    node_positions_publisher_ = this->create_publisher<march_shared_msgs::msg::StateEstimatorVisualization>("node_positions", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MarchStateEstimatorNode::publishNodePositions, this));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode constructor done");
}

void MarchStateEstimatorNode::publishNodePositions()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode::publishNodePositions");
    auto message = march_shared_msgs::msg::StateEstimatorVisualization();
    message.node_names = robot_description_->getNodeNames();
    message.parent_node_names = robot_description_->getParentNames();

    for (auto & node_position : robot_description_->getNodesPosition())
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node_position[0];
        pose.position.y = node_position[1];
        pose.position.z = node_position[2];
        message.node_poses.push_back(pose);
    }

    node_positions_publisher_->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode::publishNodePositions done");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchStateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}