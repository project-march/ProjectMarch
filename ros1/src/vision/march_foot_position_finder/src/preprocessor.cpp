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
#include <ros/console.h>

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
Preprocessor::Preprocessor(PointCloud::Ptr pointcloud,
    std::string& left_or_right, tf::TransformListener& listener)
    : pointcloud_ { std::move(pointcloud) }
{
    base_frame_ = "toes_" + left_or_right + "_aligned";

    // Define current transformation between realsense pointcloud and
    // base_frame:
    try {
        ros::Time now = ros::Time::now();
        pointcloud_frame_id_ = pointcloud_->header.frame_id.c_str();
        listener.waitForTransform(
            base_frame_, pointcloud_frame_id_, now, ros::Duration(1.0));
        listener.lookupTransform(
            base_frame_, pointcloud_frame_id_, now, transform_);

    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM(
            "Something went wrong when transforming the pointcloud: "
            << ex.what());
    }

    ros::param::get("~voxel_size", voxel_size_);
    ros::param::get("~x_min", x_min_);
    ros::param::get("~x_max", x_max_);
    ros::param::get("~y_min", y_min_);
    ros::param::get("~y_max", y_max_);
    ros::param::get("~z_min", z_min_);
    ros::param::get("~z_max", z_max_);
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
 * Filter points based on their distance using minimum and maximum allowed
 * coordinates.
 */
void Preprocessor::filterOnDistance(float x_min, float x_max, float y_min,
    float y_max, float z_min, float z_max)
{
    pcl::PointIndices::Ptr remove_indices(new pcl::PointIndices());
    pcl::ExtractIndices<Point> extract;

    for (std::size_t i = 0; i < (*pointcloud_).size(); i++) {
        Point pt = pointcloud_->points[i];
        if (pt.y <= y_min || pt.y >= y_max || pt.x <= x_min || pt.x >= x_max
            || pt.z <= z_min || pt.z >= z_max) {
            remove_indices->indices.push_back(i);
        }
    }

    extract.setInputCloud(pointcloud_);
    extract.setIndices(remove_indices);
    extract.setNegative(/*negative=*/true);
    extract.filter(*pointcloud_);
}

/**
 * Transform the realsense pointclouds to given base frame using given
 * transformations.
 */
void Preprocessor::transformPointCloudToBaseframe()
{
    pcl_ros::transformPointCloud(*pointcloud_, *pointcloud_, transform_);
    pointcloud_->header.frame_id = base_frame_;
}