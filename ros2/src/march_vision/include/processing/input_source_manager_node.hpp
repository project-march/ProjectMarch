#pragma once

#include <rclcpp/rclcpp.hpp>
#include "processing/camera_interface.h"
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> m_sync_policy;

class InputSourceManagerNode : public rclcpp::Node
{

private:
    void dualCameraCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr left_msg, 
        const sensor_msgs::msg::PointCloud2::SharedPtr right_msg);

    CameraInterface m_left_camera_interface;
    CameraInterface m_right_camera_interface;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> m_left_camera_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> m_right_camera_sub;
    std::shared_ptr<message_filters::Synchronizer<m_sync_policy>> m_sync;

public:
    InputSourceManagerNode();
    ~InputSourceManagerNode() = default;
};