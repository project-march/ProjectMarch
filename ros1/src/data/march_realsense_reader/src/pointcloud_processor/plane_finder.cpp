#include "pointcloud_processor/plane_finder.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;
using PlaneParameters = std::vector<pcl::ModelCoefficients::Ptr>;
using HullsVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;

// Construct a basic RegionCreator class
PlaneFinder::PlaneFinder(YAML::Node config_tree, bool debugging):
    config_tree_{config_tree},
    debugging_{debugging}
{

}


bool SimplePlaneFinder::find_planes(
    PointCloud::Ptr pointcloud,
    Normals::Ptr normal_pointcloud,
    boost::shared_ptr<RegionsVector> regions_vector,
    boost::shared_ptr<PlaneParameters> plane_parameters,
    boost::shared_ptr<HullsVector> hulls)
{
  pointcloud_ = pointcloud;
  normal_pointcloud_ = normal_pointcloud;
  regions_vector_ = regions_vector;
  plane_parameters_ = plane_parameters;
  hulls_ = hulls;

  // TODO: Basic algorithm to find the planes should be implemented here

  return true;
}

