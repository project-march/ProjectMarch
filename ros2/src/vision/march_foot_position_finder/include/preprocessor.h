/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;

class Preprocessor {
public:
    explicit Preprocessor(rclcpp::Node* n, std::string& left_or_right,
        std::shared_ptr<tf2_ros::TransformListener>& listener, std::shared_ptr<tf2_ros::Buffer>& buffer);

    void preprocess(const PointCloud::Ptr& pointcloud);

    void voxelDownSample(const PointCloud::Ptr& pointcloud, float voxel_size);

protected:
    void transformPointCloudToBaseframe(const PointCloud::Ptr& pointcloud);

    rclcpp::Node* n_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ { nullptr };
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ { nullptr };

    PointCloud::Ptr pointcloud_;

    std::string base_frame_;
    std::string pointcloud_frame_id_;
    geometry_msgs::msg::TransformStamped transform_;
};

#endif // MARCH_PREPROCESSOR_H
