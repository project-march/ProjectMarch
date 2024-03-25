/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_MPC_BUFFER__MARCH_MPC_BUFFER_NODE_HPP_
#define MARCH_MPC_BUFFER__MARCH_MPC_BUFFER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_ros/transform_listener.h"

class MarchMpcBufferNode : public rclcpp::Node {
public:
    MarchMpcBufferNode();
    ~MarchMpcBufferNode() = default;

private:
    void footstepsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_footsteps_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_footsteps_buffer_pub;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
};

#endif // MARCH_MPC_BUFFER__MARCH_MPC_BUFFER_NODE_HPP_