#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include "utilities/realsense_gait_utilities.h"
#include "pointcloud_processor/parameter_determiner.h"
#include "march_shared_msgs/GetGaitParameters.h"

using PlaneParameters = std::vector<pcl::ModelCoefficients::Ptr>;
using HullsVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using GaitParameters = march_shared_msgs::GetGaitParameters::Response;

ParameterDeterminer::ParameterDeterminer(YAML::Node config_tree, bool debugging):
  debugging_{debugging},
  config_tree_{config_tree}
{

}

bool SimpleParameterDeterminer::determine_parameters(
        boost::shared_ptr<PlaneParameters> plane_parameters,
        boost::shared_ptr<HullsVector> hulls,
        SelectedGait selected_obstacle,
        GaitParameters gait_parameters)
{
 return true;
};

