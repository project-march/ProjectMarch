#include "pointcloud_processor/hull_finder.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;
using PlaneParameterVector = std::vector<pcl::ModelCoefficients::Ptr>;
using HullVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using PolygonVector = std::vector<std::vector<pcl::Vertices>>;

// Construct a basic HullFinder class
HullFinder::HullFinder(YAML::Node config_tree, bool debugging):
    config_tree_{config_tree},
    debugging_{debugging}
{

}


bool SimpleHullFinder::find_hulls(
    PointCloud::Ptr pointcloud,
    Normals::Ptr normal_pointcloud,
    boost::shared_ptr<RegionVector> region_vector,
    boost::shared_ptr<PlaneParameterVector> plane_parameter_vector,
    boost::shared_ptr<HullVector> hull_vector,
    boost::shared_ptr<PolygonVector> polygon_vector)
{
  pointcloud_ = pointcloud;
  normal_pointcloud_ = normal_pointcloud;
  region_vector_ = region_vector;
  plane_parameter_vector_ = plane_parameter_vector;
  hull_vector_ = hull_vector;
  polygon_vector_ = polygon_vector;

  if (debugging_)
  {
    ROS_INFO("Finding hulls with SimpleHullFinder");
  }

  // TODO: Basic algorithm to find the planes should be implemented here

  return true;
}

