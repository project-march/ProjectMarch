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
Preprocessor::Preprocessor(rclcpp::Node* n, PointCloud::Ptr pointcloud,
    std::string& left_or_right,
    std::shared_ptr<tf2_ros::TransformListener>& listener,
    std::shared_ptr<tf2_ros::Buffer>& buffer)
    : pointcloud_ { std::move(pointcloud) }
    , n_ { n }
    , tf_buffer_ { buffer }
    , tf_listener_ { listener }
{

    base_frame_ = "toes_" + left_or_right + "_aligned";
    pointcloud_frame_id_ = pointcloud_->header.frame_id.c_str();
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

    try {
        transform_ = tf_buffer_->lookupTransform(
            base_frame_, pointcloud_frame_id_, tf2::TimePointZero);

        Eigen::Matrix<double, 3, 1> translation;
        Eigen::Quaternion<double> rotation;

        tf2::fromMsg(transform_.transform.translation, translation);
        tf2::fromMsg(transform_.transform.rotation, rotation);

        pcl::transformPointCloud(
            *pointcloud_, *pointcloud_, translation, rotation);
        pointcloud_->header.frame_id = base_frame_;

    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(
            n_->get_logger(), "Could not transform pointcloud: %s", ex.what());
    }
}