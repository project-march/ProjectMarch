#include <march_realsense_reader/realsense_reader.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

RealSenseReader::RealSenseReader(ros::NodeHandle* n):
    n_(n),
    reading_(false)
{
  pointcloud_subscriber_ = n_->subscribe<PointCloud>
    ("/camera/depth/color/points", 1,
     &RealSenseReader::pointcloud_callback, this);
  read_pointcloud_service_ = n_->advertiseService
      ("/camera/read_pointcloud",
      &RealSenseReader::read_pointcloud_callback,
      this);
}

void RealSenseReader::pointcloud_callback(const PointCloud::ConstPtr& msg)
{
  if (reading_)
  {
    // All logic to execute with a pointcloud will be executed here.
    ROS_INFO_STREAM("Processing point cloud at time " << msg->header.stamp);
    reading_ = false;
  }
}

bool RealSenseReader::read_pointcloud_callback(std_srvs::Trigger::Request &req,
                                               std_srvs::Trigger::Response &res)
{
  reading_ = true;
  res.success = true;
  return true;
}