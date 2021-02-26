#include "pointcloud_processor/region_creator.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;

RegionCreator::RegionCreator(YAML::Node config_tree):
    config_tree_{config_tree}
{

}

void SimpleRegionCreator::create_regions(PointCloud::Ptr pointcloud,
                                         Normals::Ptr normal_pointcloud,
                                         boost::shared_ptr<RegionsVector>
                                             regions_vector)
{
  pointcloud_ = pointcloud;
  normal_pointcloud_ = normal_pointcloud;
  regions_vector_ = regions_vector;
  ROS_INFO_STREAM("Creating regions with SimpleRegionCreator");
}

