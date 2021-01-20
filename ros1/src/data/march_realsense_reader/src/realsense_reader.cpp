#include <march_realsense_reader/realsense_reader.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

RealSenseReader::RealSenseReader(ros::NodeHandle* n)
    : n_(n)
{
  pointcloud_subscriber_ = n_->subscribe<PointCloud>
    ("/camera/depth/color/points", 1, &RealSenseReader::pointcloud_callback, this);
}

void RealSenseReader::pointcloud_callback(const PointCloud::ConstPtr& msg)
{
  ROS_WARN("test_callback");
}