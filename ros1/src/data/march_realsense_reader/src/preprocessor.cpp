#include <pointcloud_processor/preprocessor.h>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

Preprocessor::Preprocessor(YAML::Node config_tree,
                           PointCloud::Ptr pointcloud,
                           Normals::Ptr normal_pointcloud):
                           config_tree_{config_tree},
                           pointcloud_{pointcloud},
                           normal_pointcloud_{normal_pointcloud}
{

}

Preprocessor::Preprocessor(
    std::string file_name,
    PointCloud::Ptr pointcloud,
    Normals::Ptr normal_pointcloud):
    pointcloud_{pointcloud},
    normal_pointcloud_{normal_pointcloud}
{
  std::string path = ros::package::getPath("march_realsense_reader") +
      "/config/" + file_name;
  config_tree_ = YAML::LoadFile(path)["preprocessor"];
}

void SimplePreprocessor::preprocess() {
  ROS_INFO_STREAM("Preprocessing, test_parameter is " <<
  config_tree_["test_parameter"]);

}

void SimplePreprocessor::transformPointCloudFromUrdf() {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform("camera_link", "foot_left",
                                                ros::Time::now(), ros::Duration(0.5));
    pcl_ros::transformPointCloud(*pointcloud_, *pointcloud_,
                                 transformStamped.transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Something went wrong when transforming the poincloud: "
    << ex.what());
    return;
  }
}
