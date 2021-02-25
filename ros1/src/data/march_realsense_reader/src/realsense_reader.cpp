#include <march_realsense_reader/realsense_reader.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>
#include <pointcloud_processor/preprocessor.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

RealSenseReader::RealSenseReader(ros::NodeHandle* n):
    n_(n),
    reading_(false)
{
  pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>
    ("/camera/depth/color/points", 1,
     &RealSenseReader::pointcloud_callback, this);
  read_pointcloud_service_ = n_->advertiseService
      ("/camera/read_pointcloud",
      &RealSenseReader::read_pointcloud_callback,
      this);
  pointcloud_publisher_ = n_->advertise<PointCloud>
      ("/camera/preprocessed_cloud", 50);

  config_file_ = "pointcloud_parameters.yaml";
}

void RealSenseReader::pointcloud_callback(const sensor_msgs::PointCloud2 input_cloud)
{
//  last_pointcloud_ = input_cloud;
  if (reading_) {
    // All logic to execute with a pointcloud will be executed here.
    ROS_INFO_STREAM("Processing point cloud at time " << input_cloud.header
    .stamp);
    reading_ = false;
    PointCloud converted_cloud;
    pcl::fromROSMsg(input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud = boost::make_shared<PointCloud>(converted_cloud);
    Normals::Ptr normals = boost::make_shared<Normals>();

    std::unique_ptr<NormalsPreprocessor> preprocessor =
        std::make_unique<NormalsPreprocessor>(config_file_, pointcloud, normals);
    preprocessor->preprocess();

    pointcloud->width  = 1;
    pointcloud->height = pointcloud->points.size();

    ROS_INFO_STREAM("Done preprocessing, lets publish: " << pointcloud << " with size: " << pointcloud->points.size());
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pointcloud, msg);
    pointcloud_publisher_.publish(msg);
    ROS_INFO_STREAM("Pointcloud published");
  }
}

bool RealSenseReader::read_pointcloud_callback(std_srvs::Trigger::Request &req,
                                               std_srvs::Trigger::Response &res)
{
  reading_ = true;
  res.success = true;
  return true;
}
