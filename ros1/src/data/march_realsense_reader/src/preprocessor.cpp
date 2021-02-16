#include <pointcloud_processor/preprocessor.h>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>


using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

Preprocessor::Preprocessor(YAML::Node config_tree,
                           PointCloud::Ptr pointcloud,
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

void SimplePreprocessor::preprocess()
{
  ROS_INFO_STREAM("Preprocessing, test_parameter is " <<
  config_tree_["test_parameter"]);

}

void NormalPreprocessor::preprocess()
{
  ROS_INFO_STREAM("Preprocessing with normal filtering. Number of points BEFORE downsampling " << pointcloud_->points.size());
//  DownSample();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(pointcloud_);
  double leaf_size = config_tree_["leaf_size"].as<double>(); // How is the type of leaf_size determined?
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*pointcloud_);

  ROS_INFO_STREAM("Number of points AFTER downsampling " << pointcloud_->points.size());

}

//void DownSample()
//{
//  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
//  voxel_grid.setInputCloud(pointcloud_);
//  double leaf_size = config_tree_["leaf_size"];
//  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
//  voxel_grid.filter(*pointcloud_);
//}