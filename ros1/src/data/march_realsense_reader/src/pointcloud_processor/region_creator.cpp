#include "pointcloud_processor/region_creator.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

// Construct a basic RegionCreator class
RegionCreator::RegionCreator(YAML::Node config_tree, bool debugging):
    config_tree_{config_tree},
    debugging_{debugging}
{

}

/** This function should take in a pointcloud with matching normals and cluster them
 in regions, based on the parameters in the YAML node given at construction **/
bool SimpleRegionCreator::create_regions(PointCloud::Ptr pointcloud,
                                         Normals::Ptr normal_pointcloud,
                                         boost::shared_ptr<RegionVector>
                                             region_vector)
{
  pointcloud_ = pointcloud;
  normal_pointcloud_ = normal_pointcloud;
  region_vector_ = region_vector;
  ROS_DEBUG("Creating regions with SimpleRegionCreator");

  //TODO: Implement simple region creating algorithm
  return true;
}

