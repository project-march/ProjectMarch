#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_hull.h>
#include <ros/package.h>
#include <geometry_msgs>
#include "yaml-cpp/yaml.h"
#include "utilities/realsense_gait_utilities.h"
#include "pointcloud_processor/parameter_determiner.h"
#include "march_shared_msgs/GaitParameters.h"

using PointCloud2D = pcl::PointCloud<pcl::PointXY>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using Normals = pcl::PointCloud<pcl::Normal>;
using Region = pcl::PointIndices;
using PlaneCoefficients = pcl::ModelCoefficients;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using Polygon = std::vector<pcl::Vertices>;
using RegionVector = std::vector<Region>;
using PlaneCoefficientsVector = std::vector<PlaneCoefficients::Ptr>;
using HullVector = std::vector<Hull::Ptr>;
using PolygonVector = std::vector<Polygon>;
using GaitParameters = march_shared_msgs::GaitParameters;

ParameterDeterminer::ParameterDeterminer(YAML::Node config_tree, bool debugging):
  debugging_{debugging},
  config_tree_{config_tree}
{

}

bool HullParameterDeterminer::determine_parameters(
        boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
        boost::shared_ptr<HullVector> const hull_vector,
        boost::shared_ptr<PolygonVector> const polygon_vector,
        SelectedGait const selected_obstacle,
        bool const for_right_foot,
        boost::shared_ptr<GaitParameters> gait_parameters)
{
  ROS_DEBUG("Determining parameters with simple parameter determiner");
  hull_vector_ = hull_vector;
  selected_obstacle_ = selected_obstacle;
  gait_parameters_ = gait_parameters;
  plane_coefficients_vector_ = plane_coefficients_vector;
  polygon_vector_ = polygon_vector;
  selected_obstacle_ = selected_obstacle;
  for_right_foot_ = for_right_foot;

  gait_parameters_->step_height_parameter = 0.5;
  gait_parameters_->step_size_parameter = 0.5;

  bool success = true;

  success &= getOptimalFootLocation();

  return success;
};

bool HullParameterDeterminer::getOptimalFootLocation()
{
  bool success = true;

  // Get some locations on the ground we might want to place our foot
  PointCloud2D::Ptr foot_locations_to_try (new PointCloud2D);
  getOptionalFootLocations(foot_locations_to_try);

  // Crop those locations to only be left with locations where it is possible to place the foot
  PointNormalCloud::Ptr possible_foot_locations (new PointNormalCloud);
  succes &= cropCloudToHullVector(foot_location_to_try, possible_foot_locations);

  // Get the location where we would ideally place the foot
  getMostDesirableLocation();

  getOptimalFootLocation(possible_foot_locations);
}

bool getOptimalFootLocation(PointNormalCloud possible_foot_locations);
{
  if (possible_foot_locations->points.size() == 0)
  {

  }
  double min_distance_to_most_desirable_location = std::numeric_limits<double>::max();

  for (pcl::PointNormal possible_foot_location : *possible_foot_locations)
  {
    double distance_to_most_desirable_location = (possible_foot_location.x - most_desirable_location.x) *
                                                 (possible_foot_location.x - most_desirable_location.x) +
                                                 (possible_foot_location.y - most_desirable_location.y) *
                                                 (possible_foot_location.y - most_desirable_location.y) +
                                                 (possible_foot_location.z - most_desirable_location.z) *
                                                 (possible_foot_location.z - most_desirable_location.z);

    if (distance_to_most_desirable_location < min_distance_to_most_desirable_location)
    {
      min_distance_to_most_desirable_location = distance_to_most_desirable_location;
      optimal_foot_location = possible_foot_location;
    }
  }
}

void getMostDesirableLocation()
{
  most_desirable_foot_location.x = (min_x_stairs + max_x_stairs) / 2.0;
  most_desirable_foot_location.y = y_location;
  most_desirable_foot_location.z = (min_z_stairs + max_z_stairs) / 2.0;
}

void HullParameterDeterminer::getOptionalFootLocations(PointCloud::Ptr foot_locations_to_try)
{
  foot_location_to_try->points.resize(number_of_optional_foot_locations);
  if (selected_obstacle_ == SelectedGait.stairs_up ||
      selected_obstacle_ == SelectedGait.stairs_down)
  {
    for (int i = 0; i < number_of_optional_foot_locations; i++)
    {
      double x_location = min_x_stairs + (max_x_stairs - min_x_stairs) *
                                         i / (double) (number_of_optional_foot_locations - 1);
      foot_locations_to_try->points[i].x = x_location;
      foot_locations_to_try->points[i].y = y_location;
    }
  }
}

/** For each hull, the input cloud's z coordinate is set so that it
* lies on the corresponding plane, then the input cloud is cropped, the points inside the hull (the cropped cloud)
* are moved to the output cloud with the normal of the plane
* This process is repeated for each hull. If each point in the input_cloud has been moved to the output cloud,
* result is set to true, it is set to false otherwise **/
bool HullParameterDeterminer::cropCloudToHullVector(PointCloud2D::Ptr const input_cloud,
                                                    PointNormalCloud::Ptr output_cloud)
{
  if (input_cloud->points.size() == 0)
  {
    ROS_WARN_STREAM("cropCloudToHullVector method called with an input cloud of size zero."
                    "No cropping can be done, returning.");
    return false;
  }
  bool success = true;
  for (int hull_index = 0; hull_index < hull_vector_->size(); hull_index++)
  {
    PointCloud::Ptr elevated_cloud (new PointCloud);
    success &= addZCoordinateToCloudFromPlaneCoefficients(input_cloud,
                                                          plane_coefficients_vector_->at(hull_index),
                                                          elevated_cloud);

    success &= cropCloudToHull(elevated_cloud, hull_vector_->at(hull_index), polygon_vector_->at(hull_index));

    PointNormalCloud::Ptr elevated_cloud_with_normals (new PointNormalCloud);
    success &= addNormalToCloudFromPlaneCoefficients(elevated_cloud,
                                          plane_coefficients_vector_->at(hull_index),
                                          elevated_cloud_with_normals);

    *output_cloud += *elevated_cloud_with_normals;
  }
  return success;

}

bool HullParameterDeterminer::addZCoordinateToCloudFromPlaneCoefficients(
        PointCloud2D::Ptr const input_cloud,
        PlaneCoefficients::Ptr const plane_coefficients,
        PointCloud::Ptr elevated_cloud)
{
  if (hull_vector_->size() == 0)
  {
    ROS_ERROR_STREAM("The hull vector of the HullParameterDeterminer has size 0, cannot crop cloud to hulls.")
    return false;
  }
  elevated_cloud->points.resize(input_cloud->points.size());

  int point_index = 0;
  for (pcl::PointXY elevated_point : *elevated_cloud)
  {
    // using z = - (d + by + ax) / c from plane equation ax + by + cz + d = 0
    input_point = input_cloud->points[p]
    elevated_point.x = input_point.x;
    elevated_point.y = input_point.y;
    elevated_point.z = -(plane_coefficients->values[3] +
            plane_coefficients->values[1] * input_cloud->points[p].y +
            plane_coefficients->values[0] * input_cloud->points[p].x)
                    / plane_coefficients->values[2];
    point_index++;
  }
  return true;
}

bool HullParameterDeterminer::cropCloudToHull(
        PointCloud::Ptr elevated_cloud,
        const Hull::Ptr hull,
        const Polygon polygon)
{
  if (elevated_cloud->points.size() == 0)
  {
    ROS_WARN_STREAM("The cloud to be cropped in the HullParameterDeterminer contains no points.")
    return false;
  }
  pcl::CropHull<pcl::PointXYZ> crop_filter;
  crop_filter.setInputCloud(elevated_cloud);
  crop_filter.setHullCloud(hull);
  crop_filter.setHullIndices(polygon);
  crop_filter.setDim(2); //////////////////////////////////////////////////////////
  crop_filter.filter(*elevated_cloud);
  return true;
}

bool HullParameterDeterminer::addNormalToCloudFromPlaneCoefficients(
        PointCloud::Ptr const elevated_cloud,
        PlaneCoefficients::Ptr const plane_coefficients,
        PointNormalCloud::Ptr elevated_cloud_with_normals)
{
  elevated_cloud_with_normals->points.resize(elevated_cloud->points.size());

  double normalising_constant =
          plane_coefficients->values[0] * plane_coefficients->values[0] +
          plane_coefficients->values[1] * plane_coefficients->values[1] +
          plane_coefficients->values[2] * plane_coefficients->values[2];

  if (normalising_constant < std::numeric_limits<double>::epsilon())
  {
    ROS_ERROR_STREAM("The normal vector of the current plane is too close "
                     "to the zero vector.")
    return false;
  }

  int p = 0;
  for (pcl::PointNormal elevated_point_with_normal: *elevated_cloud_with_normals)
  {
    pcl::PointXYZ elevated_point = elevated_cloud->points[p];
    elevated_point_with_normal.x = elevated_point.x;
    elevated_point_with_normal.y = elevated_point.y;
    elevated_point_with_normal.z = elevated_point.z;

    // using that [a b c]^T is perpendicular to the plane in plane equation ax + by + cz + d = 0
    elevated_point_with_normal.normal_x = plane_coefficients->values[0] / normalising_constant;
    elevated_point_with_normal.normal_y = plane_coefficients->values[1] / normalising_constant;
    elevated_point_with_normal.normal_z = plane_coefficients->values[2] / normalising_constant;
    p++
  }
  return true;
}


bool SimpleParameterDeterminer::determine_parameters(
        boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
        boost::shared_ptr<HullVector> const hull_vector,
        boost::shared_ptr<PolygonVector> const polygon_vector,
        SelectedGait const selected_obstacle,
        boost::shared_ptr<GaitParameters> gait_parameters)
{
  ROS_DEBUG("Determining parameters with simple parameter determiner");
  hull_vector_ = hull_vector;
  selected_obstacle_ = selected_obstacle;
  gait_parameters_ = gait_parameters;
  plane_coefficients_vector_ = plane_coefficients_vector;
  polygon_vector_ = polygon_vector;

  // Return a standard step parameter, which works for medium stairs and medium ramp
  gait_parameters_->step_height_parameter = 0.5;
  gait_parameters_->step_size_parameter = 0.5;
  return true;
};