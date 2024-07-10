#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <librealsense2/rs.hpp>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>;

namespace march_vision{
    
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;

class CameraInterface {
public:
    CameraInterface(rclcpp::Node* node, const std::string& left_or_right);
    ~CameraInterface();

    void run();
    void stop();
    bool initializeCamera();
    void shutdown();

private:
    rclcpp::Node* m_node;
    std::string m_left_or_right;
    std::string m_serial_number;
    std::string m_frame_id;
    std::string m_topic_camera;
    std::clock_t m_last_frame_time;
    int m_frame_wait_counter;
    double m_frame_timeout;

    rs2::config m_config;
    rs2::pipeline m_pipeline;
    rs2::decimation_filter m_decimation_filter;
    rs2::spatial_filter m_spatial_filter;
    rs2::temporal_filter m_temporal_filter;
    rs2::hole_filling_filter m_hole_filling_filter;
    // rs2::threshold_filter m_threshold_filter;

    bool m_decimation_filter_enabled;
    bool m_spatial_filter_enabled;
    bool m_temporal_filter_enabled;
    bool m_hole_filling_filter_enabled;

    rclcpp::TimerBase::SharedPtr m_realsense_timer;
    PointCloudPublisher::SharedPtr m_preprocessed_pointcloud_publisher;
    rclcpp::CallbackGroup::SharedPtr m_realsense_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_point_callback_group;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::mutex m_mutex;

    void declareParameters();
    void readParameters();
    void processRealSenseDepthFrames();
    void processPointCloud(const PointCloud::Ptr& pointcloud);
    PointCloud::Ptr pointsToPCL(const rs2::points& points);
    void publishCloud(const PointCloudPublisher::SharedPtr& publisher, rclcpp::Node* n, PointCloud cloud);
};

} // namespace march_vision
