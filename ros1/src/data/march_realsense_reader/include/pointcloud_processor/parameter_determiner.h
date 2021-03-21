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
  virtual bool determine_parameters(
      boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
      boost::shared_ptr<HullVector> const hull_vector,
      boost::shared_ptr<PolygonVector> const polygon_vector,
      SelectedGait const selected_obstacle,
      bool const for_right_foot,
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
  bool determine_parameters(
      boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
      boost::shared_ptr<HullVector> const hull_vector,
      boost::shared_ptr<PolygonVector> const polygon_vector,
      SelectedGait const selected_obstacle,
      bool const for_right_foot,
      boost::shared_ptr<GaitParameters> gait_parameters) override;

  pcl::PointNormal optimal_foot_location;

protected:

  bool getOptimalFootLocation();

  bool getPossibleMostDesirableLocation(PointNormalCloud::Ptr possible_foot_locations);

  bool getGeneralMostDesirableLocation();

  bool getOptionalFootLocations(PointCloud2D::Ptr foot_locations_to_try);

  /** Takes a 2D point cloud of potential foot locations and returns
   * the valid foot locations with associated height and normal vector.
   * Result indicates whether every original point ends up being valid.**/
  bool cropCloudToHullVector(PointCloud2D::Ptr const input_cloud,
                             PointNormalCloud::Ptr output_cloud);

  bool addZCoordinateToCloudFromPlaneCoefficients(PointCloud2D::Ptr input_cloud,
                                                  PlaneCoefficients::Ptr plane_coefficients,
                                                  PointCloud::Ptr elevated_cloud);

  bool cropCloudToHull(PointCloud::Ptr elevated_cloud, Hull::Ptr hull, Polygon polygon);

  bool addNormalToCloudFromPlaneCoefficients(PointCloud::Ptr elevated_cloud,
                                             PlaneCoefficients::Ptr plane_coefficients,
                                             PointNormalCloud::Ptr elevated_cloud_with_normals);

  bool getGaitParametersFromFootLocation();

  bool optimalLocationIsValid();

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
  bool for_right_foot_;
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
  bool determine_parameters(
          boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
          boost::shared_ptr<HullVector> const hull_vector,
          boost::shared_ptr<PolygonVector> const polygon_vector,
          SelectedGait const selected_obstacle,
          bool const for_right_foot,
          boost::shared_ptr<GaitParameters> gait_parameters) override;
};

#endif //MARCH_PARAMETER_DETERMINER_H
