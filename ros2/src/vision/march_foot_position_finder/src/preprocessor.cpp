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
 * @param pointcloud realsense pointcloud
 * @return PointCloud::Ptr pcl pointcloud
 */
// Suppress lint error: "fields are not initialized by constructor"
// NOLINTNEXTLINE
Preprocessor::Preprocessor(
    rclcpp::Node* n, PointCloud::Ptr pointcloud, std::string& left_or_right)
    : pointcloud_ { std::move(pointcloud) }
{

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_ { nullptr };
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(n->get_clock());
    transform_listener_
        = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    base_frame_ = "toes_" + left_or_right + "_aligned";

    // Define current transformation between realsense pointcloud and
    // base_frame:
    try {
        transform_ = tf_buffer_->lookupTransform(
            base_frame_, pointcloud_frame_id_, tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {

        RCLCPP_WARN(n->get_logger(), "Could not retrieve transformation.");
    }
}

/**
 * Preprocess the current pointcloud by transforming the coordinates to world
 * frame.
 */
void Preprocessor::preprocess()
{
    transformPointCloudToBaseframe();
}

/**
 * Downsample the pointcloud using a voxel grid.
 *
 * @param voxel_size cell size of the voxel grid
 */
void Preprocessor::voxelDownSample(float voxel_size)
{
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(pointcloud_);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*pointcloud_);
}

/**
 * Transform the realsense pointclouds to given base frame using given
 * transformations.
 */
void Preprocessor::transformPointCloudToBaseframe()
{
    Eigen::Matrix<double, 3, 1> translation;
    Eigen::Quaternion<double> rotation;

    tf2::fromMsg(transform_.transform.translation, translation);
    tf2::fromMsg(transform_.transform.rotation, rotation);

    pcl::transformPointCloud(*pointcloud_, *pointcloud_, translation, rotation);
    pointcloud_->header.frame_id = base_frame_;
}