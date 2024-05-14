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

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>;


namespace march_vision{
class CameraInterface {

public:

    explicit CameraInterface(rclcpp::Node* n, const std::string& left_or_right);
    void readParameters(const std::vector<rclcpp::Parameter>& parameters);
    ~CameraInterface() = default;

protected:

    void processRealSenseDepthFrames();
    void processPointCloud(const PointCloud::Ptr& pointcloud);

    rclcpp::Node* m_node;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    rclcpp::Publisher<march_shared_msgs::msg::FootPosition>::SharedPtr m_point_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_subscriber;
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_preprocessed_pointcloud_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_point_marker_publisher;
    // TODO: How should I use these subscriptions?
    rclcpp::Subscription<march_shared_msgs::msg::FootPosition>::SharedPtr m_other_chosen_point_subscriber;
    rclcpp::Subscription<march_shared_msgs::msg::CurrentState>::SharedPtr m_current_state_subscriber;
    rclcpp::TimerBase::SharedPtr m_realsense_timer;
    rclcpp::TimerBase::SharedPtr m_initial_position_reset_timer;
    rclcpp::CallbackGroup::SharedPtr m_realsense_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_point_callback_group;
    clock_t m_last_frame_time;
    int m_frame_wait_counter;
    float m_frame_timeout;
    rs2::pipeline m_pipeline;
    rs2::config m_config;
    std::string m_serial_number;
    rs2::decimation_filter m_dec_filter;
    rs2::spatial_filter m_spat_filter;
    rs2::temporal_filter m_temp_filter;
    std::string m_topic_camera;
    std::string m_left_or_right;
    std::string m_other_frame_id;
    std::string m_current_frame_id;
    std::mutex m_mutex;
    bool m_realsense_simulation;
    double m_outlier_distance;
    double m_height_zero_threshold;
    double m_height_distance_coefficient;
    int m_switch_factor;
    int m_sample_size;
    // TODO: Do I need this?
    Point m_ORIGIN;

    PointCloud::Ptr pointsToPCL(const rs2::points& points);
    void publishCloud(const PointCloudPublisher::SharedPtr& publisher, rclcpp::Node* n, PointCloud cloud, std::string& left_or_right);

};
} // namespace march_vision
