#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "utilities/realsense_gait_utilities.h"
#include "pointcloud_processor/parameter_determiner.h"
#include "march_shared_msgs/GaitParameters.h"

using PointCloud2D = pcl::PontCloud<pcl::PointXY>:
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

ParameterDeterminer::ParameterDeterminer(YAML::Node config_tree, bool debugging):
  debugging_{debugging},
  config_tree_{config_tree}
{

}

bool SimpleParameterDeterminer::determine_parameters(
        boost::shared_ptr<PlaneParameterVector> const plane_parameter_vector,
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

};

/** For each hull, the input cloud's z coordinate is set so that it
* lies on the corresponding plane, then the input cloud is cropped, the points inside the hull (the cropped cloud)
* are moved to the output cloud with the normal of the plane
* This process is repeated for each hull. If each point in the input_cloud has been moved to the output cloud,
* result is set to true, it is set to false otherwise **/
bool HullParameterDeterminer::cropCloudToHullVector(PointCloud2D::Ptr const input_cloud,
                                                    Normals::Ptr output_cloud,
                                                    bool result)
{
  bool success = true;
  for (int hull_index = 0; hull_index < hull_vector_->size(); hull_index++){
    PointCloud::Ptr elevated_cloud (new PointCloud);
    addZCoordinateToCloudFromPlaneCoefficients(input_cloud,
                                               plane_coefficients_vector_->at(hull_index),
                                               elevated_cloud)
  }
}

bool HullParameterDeterminer::addZCoordinateToCloudFromPlaneCoefficients()

bool SimpleParameterDeterminer::determine_parameters(
        boost::shared_ptr<PlaneParameterVector> const plane_parameter_vector,
        boost::shared_ptr<HullVector> const hull_vector,
        boost::shared_ptr<PolygonVector> const polygon_vector,
        SelectedGait const selected_obstacle,
        boost::shared_ptr<GaitParameters> gait_parameters)
{
  ROS_DEBUG("Determining parameters with simple parameter determiner");
  hull_vector_ = hull_vector;
  selected_obstacle_ = selected_obstacle;
  gait_parameters_ = gait_parameters;
  plane_parameter_vector_ = plane_parameter_vector;
  polygon_vector_ = polygon_vector;

  // Return a standard step parameter, which works for medium stairs and medium ramp
  gait_parameters_->step_height_parameter = 0.5;
  gait_parameters_->step_size_parameter = 0.5;
  return true;
};