#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "utilities/realsense_gait_utilities.h"
#include "pointcloud_processor/parameter_determiner.h"
#include "march_shared_msgs/GaitParameters.h"

using PlaneParameterVector = std::vector<pcl::ModelCoefficients::Ptr>;
using HullVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using GaitParameters = march_shared_msgs::GaitParameters;
using PolygonVector = std::vector<std::vector<pcl::Vertices>>;


ParameterDeterminer::ParameterDeterminer(YAML::Node config_tree, bool debugging):
  debugging_{debugging},
  config_tree_{config_tree}
{

}

bool SimpleParameterDeterminer::determine_parameters(
        boost::shared_ptr<PlaneParameterVector> plane_parameter_vector,
        boost::shared_ptr<HullVector> hull_vector,
        boost::shared_ptr<PolygonVector> polygon_vector,
        SelectedGait selected_obstacle,
        boost::shared_ptr<GaitParameters> gait_parameters)
{
  if (debugging_)
  {
    ROS_INFO("Determining parameters with simple parameter determiner");
  }
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

