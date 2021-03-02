#include <march_realsense_reader/realsense_reader.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <march_shared_msgs/GetGaitParameters.h>
#include <pointcloud_processor/preprocessor.h>
#include <pointcloud_processor/region_creator.h>
#include <pointcloud_processor/parameter_determiner.h>
#include <pointcloud_processor/plane_finder.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;
using PlaneParameters = std::vector<pcl::ModelCoefficients::Ptr>;
using HullsVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;

std::string POINTCLOUD_TOPIC = "/camera/depth/color/points";

RealSenseReader::RealSenseReader(ros::NodeHandle* n):
    n_(n)
{
  pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>
      (POINTCLOUD_TOPIC, 1,
       &RealSenseReader::pointcloud_callback, this);
  read_pointcloud_service_ = n_->advertiseService
      ("/camera/process_pointcloud",
       &RealSenseReader::process_pointcloud_callback,
       this);

  config_tree_ = readConfig("pointcloud_parameters.yaml");

  if (config_tree_["debug"])
  {
    debugging_ = config_tree_["debug"].as<bool>();
  }
  else
  {
    debugging_ = false;
  }

  preprocessor_ = std::make_unique<NormalsPreprocessor>(
      getConfigIfPresent("preprocessor"), debugging_);
  region_creator_ = std::make_unique<SimpleRegionCreator>(
      getConfigIfPresent("region_creator"), debugging_);
  plane_finder_ = std::make_unique<SimplePlaneFinder>(
      getConfigIfPresent("plane_finder"), debugging_);
  parameter_determiner_ = std::make_unique<SimpleParameterDeterminer>(
      getConfigIfPresent("parameter_determiner"), debugging_);


  if (debugging_)
  {
    ROS_INFO("Realsense reader started with debugging, all intermediate result "
             "steps will be published and more information given in console, but"
             " this might slow the process, this can be turned off in the yaml.");
    preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>
        ("/camera/preprocessed_cloud", 1);
  }
}

YAML::Node RealSenseReader::readConfig(std::string config_file) {
  YAML::Node config_tree;
  std::string path = ros::package::getPath("march_realsense_reader") +
                     "/config/" + config_file;
  try
  {
    config_tree = YAML::LoadFile(path);
  }
  catch (YAML::Exception e)
  {
    ROS_WARN_STREAM("YAML file with path " << path << " could not be loaded, using "
                                                      "empty config instead");
  }
  return config_tree;
}

YAML::Node RealSenseReader::getConfigIfPresent(std::string key)
{
  if (config_tree_[key])
  {
    return config_tree_[key];
  }
  else
  {
    ROS_WARN_STREAM("Key " << key << " was not found in the config file, empty config "
                                     "will be used");
    return YAML::Node();
  }
}

// This method executes the logic to process a pointcloud
bool RealSenseReader::process_pointcloud(
    PointCloud::Ptr pointcloud,
    int selected_gait, march_shared_msgs::GetGaitParameters::Response &res)
{
  Normals::Ptr normals = boost::make_shared<Normals>();

  // Preprocess
  bool preprocessing_was_successful = preprocessor_->preprocess(pointcloud, normals);
  if (not preprocessing_was_successful)
  {
    res.error_message = "Preprocessing was unsuccessful, see debug output "
                        "for more information";
    return false;
  }
  if (debugging_)
  {
    ROS_INFO("Done preprocessing, see /camera/preprocessed_cloud for results");
    publishPreprocessedPointCloud(pointcloud);
  }

  // Create regions
  boost::shared_ptr<RegionsVector> regions_vector =
      boost::make_shared<RegionsVector>();
  bool region_creating_was_successful =
      region_creator_->create_regions(pointcloud, normals, regions_vector);
  if (not region_creating_was_successful)
  {
    res.error_message = "Region creating was unsuccessful, see debug output "
                        "for more information";
    return false;
  }
  if (debugging_)
  {
    ROS_INFO("Done creating regions");
    //TODO: Add publisher to visualize created regions
  }

  // Find planes
  boost::shared_ptr<PlaneParameters> plane_parameters =
      boost::make_shared<PlaneParameters>();
  boost::shared_ptr<HullsVector> hulls = boost::make_shared<HullsVector>();

  res.success = true;
  return true;
}

// Publishes the pointcloud on a topic for visualisation in rviz or further use
void RealSenseReader::publishPreprocessedPointCloud(PointCloud::Ptr pointcloud)
{
  ROS_INFO_STREAM("Publishing a preprocessed cloud with size: " << pointcloud->points.size());

  pointcloud->width  = 1;
  pointcloud->height = pointcloud->points.size();

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*pointcloud, msg);

  preprocessed_pointcloud_publisher_.publish(msg);
}

// The callback for the service that was starts processing the point cloud and gives
// back parameters for a gait
bool RealSenseReader::process_pointcloud_callback(
    march_shared_msgs::GetGaitParameters::Request &req,
    march_shared_msgs::GetGaitParameters::Response &res)
{
  boost::shared_ptr<const sensor_msgs::PointCloud2> input_cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>
      (POINTCLOUD_TOPIC, *n_, ros::Duration(1.0));

  if (input_cloud == nullptr)
  {
    res.error_message = "No pointcloud published within 1 second, so "
                        "no processing could be done";
    return false;
  }

  PointCloud converted_cloud;
  pcl::fromROSMsg(*input_cloud, converted_cloud);
  PointCloud::Ptr point_cloud = boost::make_shared<PointCloud>(converted_cloud);

  return process_pointcloud(point_cloud, req.selected_gait, res);
}