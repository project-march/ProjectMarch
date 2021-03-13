#include "pointcloud_processor/hull_finder.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <utilities/output_utilities.h>
#include <cmath>

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
     boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector,
     boost::shared_ptr<HullVector> hull_vector,
     boost::shared_ptr<PolygonVector> polygon_vector)
{
  pointcloud_ = pointcloud;
  normal_pointcloud_ = normal_pointcloud;
  region_vector_ = region_vector;
  plane_coefficients_vector_ = plane_coefficients_vector;
  hull_vector_ = hull_vector;
  polygon_vector_ = polygon_vector;

  ROS_DEBUG("Finding hulls with CHullFinder, C for convex or concave");

  bool success = true;

  succes &= readYaml();

  succes &= initializeOutputVariables();

  for (region_index_ = 0; region_index_ < region_vector_length_; region_index++)
  {
    succes &= getCHullFromRegion();
  }

  return true;
}

// Read all relevant parameters from the parameter yaml file
bool CHullFinder::readYaml()
{

}

bool CHullFinder::initializeOutputVariables()
{
  region_vector_length_ = region_vector->size();

  plane_coefficients_vector->resize(region_vector_length_);
  hull_vector_->resize(region_vector_length_);
  polygon_vector_->resize(region_vector_length_);
//  for (int index = 0; index < region_vector_length_; index++){
//    // Plane coefficients given as [0]*x + [1]*y + [2]*z + [3] = 0
//    PlaneCoefficients::Ptr plane_coefficients (new PlaneCoefficients);
//    plane_coefficients->values.resize(4);
//
//    Hull hull;
//
//    Polygon polygon;
//
//    plane_coefficients_vector_[index] = plane_coefficients;
//    hull_vector_[index] = hull;
//    polygon_vector_[index] = polygon;
//
//  }
}

// Converts a region into a convex or concave hull
bool CHullFinder::getCHullFromRegion()
{
  bool success = true;

  // Select the region from the cloud
  pcl::copyPointCloud(*pointcloud_, clusters_indices[region_index_], *region_points_);
  pcl::copyPointCloud(*pointcloud_norals_, clusters_indices[region_index_], *region_normals_);

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
  // calculate average normal and average point to calculate plane coefficients from
  std::vector<double> average_normal(3);
  std::vector<double> average_point(3);

  bool success = getAveragePointAndNormal(average_point, average_normal);

  // Plane coefficients given as [0]*x + [1]*y + [2]*z + [3] = 0
  // The first three coefficients are the normal vector of the plane as
  // all the vectors in the plane are perpendicular to the normal
  plane_coefficients_->values[0] = average_normal[0];
  plane_coefficients_->values[1] = average_normal[1];
  plane_coefficients_->values[2] = average_normal[2];
  // the final coefficient can be calculated with the plane equation ax + by + cz + d = 0
  plane_coefficients_->values[3] = - (
      average_point[0] * average_normal[0] +
      average_point[1] * average_normal[1] +
      average_point[2] * average_normal[2]);

  ROS_DEBUG_STREAM("Region " << region_index_ << " has plane coefficients: " <<
                   output_utilities::vectorToString(plane_coefficients->values));

  return success;
}

// project the region to its plane so a convex or concave hull can be created
bool CHullFinder::projectRegionToPlane()
{
  // Create the projection object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (region_points_);
  proj.setModelCoefficients (plane_coefficients_);
  proj.filter (*region_points_projected_);
  return true;
}

// Create the convex or concave hull from a projected region and its corresponding polygons
bool CHullFinder::getCHullFromProjectedPlane()
{
  if (convex)
  {
    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud (region_points_projected_);
    convex_hull.setDimension(hull_dimension);
    convex_hull.reconstruct(hull_, polygon_);
  }
  else
  {
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setAlpha(alpha);
    concave_hull.setInputCloud (region_points_projected_);
    concave_hull.setDimension(hull_dimension);
    convex_hull.reconstruct(hull_, polygon_);
  }
  return true;
}

// Add the hull to a vector together with its plane coefficients and polygons
bool CHullFinder::addCHullToVector()
{
  hull_vector_[i] = hull_;
  polygon_vector_[i] = polygon_;
  plane_coefficients_vector_[i] = plane_coefficients_;
  return true;
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

    // If the normal is zero (<=> it has a norm of zero) it is invalid
    // and it then cannot be used to calculate plane coefficients.
    // The norm is generally close to 1, but this need not be the case
    // e.g. if the orientation of the normals is not consistent.
    float minimum_norm_allowed = 0.05;
    if (average_normals[0] * average_normals[0] +
        average_normals[1] * average_normals[1] +
        average_normals[2] * average_normals[2] > minimum_norm_allowed)
    {
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Region with index " << region_index_ << " does not have the same number of points and normals, "
                     " unable to calculate average point and normal. "
                     "Number of points: " << number_of_points << ". Number of normals: " << number_of_normals);
    return false;
  }

  return true;
}