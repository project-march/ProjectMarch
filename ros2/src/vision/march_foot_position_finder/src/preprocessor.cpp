/**
 * @author Tuhin Das - MARCH 7
 */

#include <chrono>
#include <cmath>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <preprocessor.h>
#include <tf2_eigen/tf2_eigen.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using Normal = pcl::Normal;
using NormalCloud = pcl::PointCloud<Normal>;

/**
 * Constructs a preprocessor object.
 *
 * @param n ROS node instance.
 * @param left_or_right Whether to find left or right foot points.
 * @param listener ROS Transform listener.
 * @param buffer ROS transform buffer.
 */
Preprocessor::Preprocessor(rclcpp::Node* n, std::string& left_or_right,
    std::shared_ptr<tf2_ros::TransformListener>& listener, std::shared_ptr<tf2_ros::Buffer>& buffer)
    : n_ { n }
    , tf_buffer_ { buffer }
    , tf_listener_ { listener }
{
    base_frame_ = "toes_" + left_or_right + "_aligned";
    pointcloud_frame_id_ = "camera_front_" + left_or_right + "_virtual_rotated";
}

/**
 * Preprocess the current pointcloud by transforming the coordinates to world
 * frame.
 *
 * @param pointcloud Pointcloud to preprocess.
 */
void Preprocessor::preprocess(const PointCloud::Ptr& pointcloud)
{
    transformPointCloudToBaseframe(pointcloud);
}

/**
 * Downsample the pointcloud using a voxel grid.
 *
 * @param pointcloud Pointcloud to downsample.
 * @param voxel_size cell size of the voxel grid
 */
void Preprocessor::voxelDownSample(const PointCloud::Ptr& pointcloud, float voxel_size)
{
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(pointcloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*pointcloud);
}

/**
 * Transform the realsense pointclouds to given base frame using given
 * transformations.
 *
 * @param pointcloud Pointcloud to transform.
 */
void Preprocessor::transformPointCloudToBaseframe(const PointCloud::Ptr& pointcloud)
{
    try {
        transform_ = tf_buffer_->lookupTransform(base_frame_, pointcloud_frame_id_, tf2::TimePointZero);

        Eigen::Matrix<double, 3, 1> translation;
        Eigen::Quaternion<double> rotation;

        tf2::fromMsg(transform_.transform.translation, translation);
        tf2::fromMsg(transform_.transform.rotation, rotation);

        pcl::transformPointCloud(*pointcloud, *pointcloud, translation, rotation);
        pointcloud->header.frame_id = base_frame_;

    } catch (tf2::TransformException& ex) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(n_->get_logger(), steady_clock, 4000, "Could not transform pointcloud: %s", ex.what());
    }
}