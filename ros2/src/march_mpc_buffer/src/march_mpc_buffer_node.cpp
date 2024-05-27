/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_mpc_buffer/march_mpc_buffer_node.hpp"

#include <string>
#include <vector> 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

MarchMpcBufferNode::MarchMpcBufferNode() : Node("march_mpc_buffer_node") {
    m_footsteps_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "final_feet_position", 10, std::bind(&MarchMpcBufferNode::footstepsCallback, this, std::placeholders::_1));
    m_footsteps_buffer_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("mpc_solver/buffer/output", 10);

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void MarchMpcBufferNode::footstepsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // Transform the footsteps from the right foot frame to the backpack frame.
    geometry_msgs::msg::PoseArray transformed_footsteps;
    transformed_footsteps.header.frame_id = "backpack"; 
    transformed_footsteps.header.stamp = msg->header.stamp; 

    for (auto& pose : msg->poses) {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = m_tf_buffer->lookupTransform("backpack", "R_heel", tf2::TimePointZero, tf2::durationFromSec(0.1));
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        tf2::Transform tf_right_foot_to_backpack;
        tf2::fromMsg(transform.transform, tf_right_foot_to_backpack);

        tf2::Transform tf_right_foot;
        tf2::fromMsg(pose, tf_right_foot);

        tf2::Transform tf_backpack = tf_right_foot_to_backpack * tf_right_foot;

        geometry_msgs::msg::Pose pose_backpack;
        tf2::toMsg(tf_backpack, pose_backpack);

        // Remove the orientation of the backpack frame.
        pose_backpack.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

        transformed_footsteps.poses.push_back(pose_backpack);
    }

    m_footsteps_buffer_pub->publish(transformed_footsteps);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchMpcBufferNode>());
    rclcpp::shutdown();
    return 0;
}