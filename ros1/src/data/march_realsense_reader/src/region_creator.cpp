#include "pointcloud_processor/region_creator.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;

RegionCreator::RegionCreator(YAML::Node config_tree,
                           PointCloud::Ptr pointcloud,
                           Normals::Ptr normal_pointcloud,
                           boost::shared_ptr<RegionsVector> regions_vector):
    config_tree_{config_tree},
    pointcloud_{pointcloud},
    normal_pointcloud_{normal_pointcloud},
    regions_vector_{regions_vector}
{

}

void SimpleRegionCreator::create_regions()
{
  ROS_INFO_STREAM("Creating regions with SimpleRegionCreator");
}

