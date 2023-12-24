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

    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "new_joint_states", 10, std::bind(&MarchStateEstimatorNode::jointStateCallback, this, std::placeholders::_1));
    node_positions_publisher_ = this->create_publisher<march_shared_msgs::msg::StateEstimatorVisualization>("node_positions", 10);
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MarchStateEstimatorNode::publishNodePositions, this));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode constructor done");
}

void MarchStateEstimatorNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode::jointStateCallback");
    // for (auto & robot_node : robot_description_->getRobotNodes())
    // {
    //     if (robot_node->getType() == 'J')
    //     {
    //         auto robot_joint = dynamic_cast<RobotJoint*>(robot_node);
    //         for (auto & name : msg->name)
    //         {
    //             if (robot_joint->getName() == name)
    //             {
    //                 robot_joint->setAngle(msg->position[std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), name))]);
    //             }
    //         }
    //     }
    // }

    auto message = march_shared_msgs::msg::StateEstimatorVisualization();
    message.node_names = robot_description_->getNodeNames();
    message.parent_node_names = robot_description_->getParentNames();

    for (auto & node_position : robot_description_->getNodesPosition(msg->name, msg->position))
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node_position[0];
        pose.position.y = node_position[1];
        pose.position.z = node_position[2];
        message.node_poses.push_back(pose);
    }

    node_positions_publisher_->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode::jointStateCallback done");
}

// void MarchStateEstimatorNode::publishNodePositions()
// {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode::publishNodePositions");
//     auto message = march_shared_msgs::msg::StateEstimatorVisualization();
//     message.node_names = robot_description_->getNodeNames();
//     message.parent_node_names = robot_description_->getParentNames();

//     for (auto & node_position : robot_description_->getNodesPosition())
//     {
//         geometry_msgs::msg::Pose pose;
//         pose.position.x = node_position[0];
//         pose.position.y = node_position[1];
//         pose.position.z = node_position[2];
//         message.node_poses.push_back(pose);
//     }

//     node_positions_publisher_->publish(message);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MarchStateEstimatorNode::publishNodePositions done");
// }

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchStateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}