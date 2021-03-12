#include "pointcloud_processor/hull_finder.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <utilities/output_utilities.h>

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
  success &= getPlaneCoefficientsRegion();

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
  // calculate average normal and average point to calculate plane parameters from
  std::vector<double> average_normal(3);
  std::vector<double> average_point(3);

  bool success = getAveragePointAndNormal(average_point, average_normal);

  // Plane parameters given as [0]*x + [1]*y + [2]*z + [3] = 0
  // The first three parameters are the normal vector of the plane as
  // all the vectors in the plane are perpendicular to the normal
  plane_coefficients->values[0] = average_normal[0];
  plane_coefficients->values[1] = average_normal[1];
  plane_coefficients->values[2] = average_normal[2];
  // the final parameter can be calculated with the plane equation ax + by + cz + d = 0
  plane_coefficients->values[3] = - (
      average_point[0] * average_normal[0] +
      average_point[1] * average_normal[1] +
      average_point[2] * average_normal[2]);

  ROS_DEBUG_STREAM("Region " << region_index << " has plane coefficients: " <<
                   output_utilities::vectorToString(plane_coefficients->values));

  return success;
}

// project the region to its plane so a convex or concave hull can be created
bool CHullFinder::projectRegionToPlane()
{
  bool success = true;

  return success;
}

// Create the convex or concave hull from a projected region and its corresponding polygons
bool CHullFinder::getCHullFromProjectedPlane()
{
  bool success = true;

  return success;
}

// Add the hull to a vector together with its plane coefficients and polygons
bool CHullFinder::addCHullToVector()
{
  bool success = true;

  return success;
}

// Calculate the average normal and point of a region
bool CHullFinder::getAveragePointAndNormal(std::vector<double> average_point,
                                           std::vector<double> average_normal)
{
  std::fill(average_point.begin(), average_point.end(), 0);
  std::fill(average_normal.begin(), average_normal.end(), 0);

  int number_of_points = region_points_->points.size();
  int number_of_normals = region_normals_->poitns.size();
  if (number_of_points == number_of_normals)
  {
    for (int p = 0; p < number_of_points; p++)
    {
      current_normal = region_normals_->points[p];
      current_point = region_points_->point[p];

      average_normals[0] += current_normal.normal_x;
      average_normals[1] += current_normal.normal_y;
      average_normals[2] += current_normal.normal_z;
      average_point[0] += current_poitns_.x;
      average_point[1] += current_poitns_.y;
      average_point[2] += current_poitns_.z;
    }

    average_normals[0] /= number_of_normals;
    average_normals[1] /= number_of_normals;
    average_normals[2] /= number_of_normals;

    average_point[0] /= number_of_points;
    average_point[1] /= number_of_points;
    average_point[2] /= number_of_points;
  }
  else
  {
    ROS_ERROR_STREAM("Region with index " << region_index << " does not have the same number of points and normals, "
                     " unable to calculate average point and normal. "
                     "Number of points: " << number_of_points << ". Number of normals: " << number_of_normals);
    return false;
  }

  return true;
}