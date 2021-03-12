#include "pointcloud_processor/hull_finder.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using Region = pcl::PointIndices;
using PlaneCoefficients = pcl::ModelCoefficients;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using Polygon = std::vector<pcl::Vertices>;
using RegionVector = std::vector<Region>;
using PlaneCoefficientsVector = std::vector<PlaneCoefficients::Ptr>;
using HullVector = std::vector<Hull>;
using PolygonVector = std::vector<Polygon>;

// Construct a basic HullFinder class
HullFinder::HullFinder(YAML::Node config_tree, bool debugging):
    config_tree_{config_tree},
    debugging_{debugging}
{

}

bool CHullFinder::find_hulls(
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

  ROS_DEBUG("Finding hulls with CHullFinder, C for convex or concave");

  bool success = true;

  succes &= getYaml();

  succes &= initializeVariables();

  for (region_index = 0; region_index < region_vector_length; region_index++)
  {
    succes &= getCHullFromRegion();
  }

  return true;
}

bool CHullFinder::getYaml()
{

}

bool CHullFinder::initializeVariables()
{
  region_vector_length = region_vector.size();
  plane_parameter_vector.resize(region_vector_length);
  hull_vector_.resize(region_vector_length);
  polygon_vector_.resize(region_vector_length);

  for (int index = 0; index < region_vector_length; index++){
    // Plane parameters given as [0]*x + [1]*y + [2]*z + [3] = 0
    PlaneCoefficients::Ptr plane_coefficients (new PlaneCoefficients);
    plane_coefficients->values.resize(4);
    plane_parameter_vector_[index] = plane_coefficient
  }
}

// Converts a region into a convex or concave hull
bool CHullFinder::getCHullFromRegion()
{
  bool success = true;
  // Select the region from the cloud
  pcl::copyPointCloud(*pointcloud_, clusters_indices[region_index], *region_points_);
  pcl::copyPointCloud(*pointcloud_norals_, clusters_indices[region_index], *region_normals_);

  // Get the plane coefficients of the region
  success &= getPlaneCoefficinetsRegion();

  // Project the region to its plane so a convex or concave hull can be created
  succes &= projectRegionToPlane();

  // Create the hull
  succes &= getCHullFromProjectedPlane();

  // Add the hull to a vector together with its plane coefficients and polygons
  succes &= addCHullToVector();

  return success
}

// Get the plane coefficients of the region using average point and normal
bool CHullFinder::getPlaneCoefficientsRegion()
{
  bool succes = true;
  plane_coefficients->values.resize (4);

  return succes;
}

// project the region to its plane so a convex or concave hull can be created
bool CHullFinder::projectRegionToPlane()
{
  bool succes = true;

  return succes;
}

// Create the convex or concave hull from a projected region and its corresponding polygons
bool CHullFinder::getCHullFromProjectedPlane()
{
  bool succes = true;

  return succes;
}

// Add the hull to a vector together with its plane coefficients and polygons
bool CHullFinder::addCHullToVector()
{
  bool succes = true;

  return succes;
}