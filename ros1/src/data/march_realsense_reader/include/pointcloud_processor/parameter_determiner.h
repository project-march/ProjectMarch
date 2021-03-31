#ifndef MARCH_PARAMETER_DETERMINER_H
#define MARCH_PARAMETER_DETERMINER_H
#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "utilities/realsense_gait_utilities.h"
#include "march_shared_msgs/GetGaitParameters.h"

using PointCloud2D = pcl::PointCloud<pcl::PointXY>;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
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

class ParameterDeterminer
{
public:
  ParameterDeterminer(YAML::Node config_tree, bool debugging);
  /** This function is required to be implemented by any plane finder **/
  virtual bool determineParameters(
      boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
      boost::shared_ptr<HullVector> const hull_vector,
      boost::shared_ptr<PolygonVector> const polygon_vector,
      SelectedGait const selected_obstacle,
      boost::shared_ptr<GaitParameters> gait_parameters)=0;

  virtual ~ParameterDeterminer() {};

protected:
  boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector_;
  boost::shared_ptr<HullVector> hull_vector_;
  boost::shared_ptr<PolygonVector> polygon_vector_;
  SelectedGait selected_obstacle_;
  boost::shared_ptr<GaitParameters> gait_parameters_;
  YAML::Node config_tree_;
  bool debugging_;
};

/** The hull parameter determiner
 *
 */
class HullParameterDeterminer : ParameterDeterminer
{
public:
  /** Basic constructor for ParameterDeterminer preprocessor, but this will also read the yaml **/
  HullParameterDeterminer(YAML::Node config_tree, bool debugging);

  /** This function should take in a pointcloud with matching normals and
  * hulls, and turn this into a location where the foot can be placed,
  * from this location, gaits parameters should be made. **/
  bool determineParameters(
      boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
      boost::shared_ptr<HullVector> const hull_vector,
      boost::shared_ptr<PolygonVector> const polygon_vector,
      SelectedGait const selected_obstacle,
      boost::shared_ptr<GaitParameters> gait_parameters) override;

  pcl::PointNormal optimal_foot_location;
  PointNormalCloud::Ptr possible_foot_locations;
  PointCloud2D::Ptr foot_locations_to_try;


protected:
  // Get the optimal foot location by finding which possible foot location is closest
  // to the most desirable foot location
  bool getOptimalFootLocation();

  // From the possible foot locations, find which one is closes to the most desirable location
  bool getPossibleMostDesirableLocation(PointNormalCloud::Ptr possible_foot_locations);

  // Compute the optimal foot location as if one were not limited by anything.
  bool getGeneralMostDesirableLocation();

  // Create a point cloud with points on the ground where the points represent
  // where it should be checked if there is a valid foot location
  bool getOptionalFootLocations(PointCloud2D::Ptr foot_locations_to_try);

  /** Takes a 2D point cloud of potential foot locations and returns
   * the valid foot locations with associated height and normal vector.
   * Result indicates whether every original point ends up being valid.**/
  bool cropCloudToHullVector(PointCloud2D::Ptr const input_cloud,
                             PointNormalCloud::Ptr output_cloud);

  // Elevate the 2D points so they have z coordinate as if they lie on the plane of the hull
  bool addZCoordinateToCloudFromPlaneCoefficients(PointCloud2D::Ptr input_cloud,
                                                  PlaneCoefficients::Ptr plane_coefficients,
                                                  PointCloud::Ptr elevated_cloud);

  // Remove all points from a cloud which do not fall in the hull
  bool cropCloudToHull(PointCloud::Ptr elevated_cloud, Hull::Ptr hull, Polygon polygon);

  // Add normals to the elevated cloud which correspond to the normal vector of the plane
  bool addNormalToCloudFromPlaneCoefficients(PointCloud::Ptr elevated_cloud,
                                             PlaneCoefficients::Ptr plane_coefficients,
                                             PointNormalCloud::Ptr elevated_cloud_with_normals);

  // Find the parameters from the foot location by finding at what percentage of the end points it is
  bool getGaitParametersFromFootLocation();

  // Verify that a possible foot location is valid for the requested gait
  bool isValidLocation(pcl::PointNormal possible_foot_location);

  // Read all relevant parameters from the parameter yaml file
  void readYaml();
  int number_of_optional_foot_locations;
  double min_x_stairs;
  double max_x_stairs;
  double min_z_stairs;
  double max_z_stairs;
  double y_location;
  bool general_most_desirable_location_is_mid;
  bool general_most_desirable_location_is_small;

  SelectedGait selected_obstacle_;
  pcl::PointXYZ most_desirable_foot_location_;
};

/** The simple parameter determiner
 *
 */
class SimpleParameterDeterminer : ParameterDeterminer
{
public:
  /** Use the constructors defined in the super class **/
  using ParameterDeterminer::ParameterDeterminer;
  /** A Simple implementation which return parameters of 0.5 **/
  bool determineParameters(
          boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
          boost::shared_ptr<HullVector> const hull_vector,
          boost::shared_ptr<PolygonVector> const polygon_vector,
          SelectedGait const selected_obstacle,
          boost::shared_ptr<GaitParameters> gait_parameters) override;
};

#endif //MARCH_PARAMETER_DETERMINER_H
