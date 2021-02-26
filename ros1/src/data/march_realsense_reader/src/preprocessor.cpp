#include <pointcloud_processor/preprocessor.h>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

Preprocessor::Preprocessor(YAML::Node config_tree):
                           config_tree_{config_tree}
{

}

SimplePreprocessor::SimplePreprocessor(YAML::Node config_tree):
    Preprocessor(config_tree)
{
  tfBuffer = std::make_unique<tf2_ros::Buffer>();
  tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
}

void SimplePreprocessor::preprocess(PointCloud::Ptr pointcloud,
                                    Normals::Ptr normal_pointcloud) {
  pointcloud_ = pointcloud;
  normal_pointcloud_ = normal_pointcloud;
  ROS_INFO_STREAM("Preprocessing, test_parameter is " <<
  config_tree_["test_parameter"]);

  transformPointCloudFromUrdf();
}

void SimplePreprocessor::transformPointCloudFromUrdf() {
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer->lookupTransform("camera_link", "foot_left",
                                                ros::Time::now(), ros::Duration(0.5));
    pcl_ros::transformPointCloud(*pointcloud_, *pointcloud_,
                                 transformStamped.transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Something went wrong when transforming the pointcloud: "
                        << ex.what());
    return;
  }
}
