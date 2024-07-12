/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_mpc_buffer/march_mpc_buffer_node.hpp"

#include <string>
#include <vector> 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

MarchMpcBufferNode::MarchMpcBufferNode() : Node("march_mpc_buffer_node") {
    m_footsteps_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "final_feet_position", 10, std::bind(&MarchMpcBufferNode::footstepsCallback, this, std::placeholders::_1));
    m_com_trajectory_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "com_trajectory", 10, std::bind(&MarchMpcBufferNode::comTrajectoryCallback, this, std::placeholders::_1));

    m_footsteps_buffer_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("mpc_solver/buffer/output", 10);
    m_com_buffer_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("mpc_solver/buffer/com_output", 10);

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void MarchMpcBufferNode::footstepsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // Transform the footsteps from the right foot frame to the backpack frame.
    geometry_msgs::msg::PoseArray transformed_footsteps;
    transformed_footsteps.header.frame_id = "backpack"; 
    transformed_footsteps.header.stamp = msg->header.stamp; 

    for (long unsigned int i = 0; i < msg->poses.size(); i++) {
        auto pose = msg->poses[i];
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = m_tf_buffer->lookupTransform("backpack", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
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

void MarchMpcBufferNode::comTrajectoryCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // Transform the first COM pose from the right foot frame to the backpack frame.
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = m_tf_buffer->lookupTransform("world", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }

    tf2::Transform tf_right_foot_to_backpack;
    tf2::fromMsg(transform.transform, tf_right_foot_to_backpack);

    tf2::Transform tf_right_foot;
    tf2::fromMsg(msg->poses[1], tf_right_foot);

    tf2::Transform tf_backpack = tf_right_foot_to_backpack * tf_right_foot;

    geometry_msgs::msg::Pose transformed_com_pose;
    tf2::toMsg(tf_backpack, transformed_com_pose);

    // Remove the orientation of the backpack frame.
    transformed_com_pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

    geometry_msgs::msg::PoseStamped transformed_com_pose_stamped;
    transformed_com_pose_stamped.header.frame_id = "world";
    transformed_com_pose_stamped.header.stamp = msg->header.stamp;
    transformed_com_pose_stamped.pose = transformed_com_pose;

    m_com_buffer_pub->publish(transformed_com_pose_stamped);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchMpcBufferNode>());
    rclcpp::shutdown();
    return 0;
}