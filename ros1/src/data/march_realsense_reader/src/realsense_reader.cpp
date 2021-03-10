#include <ctime>
#include <march_realsense_reader/realsense_reader.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <march_shared_msgs/GetGaitParameters.h>
#include <pointcloud_processor/preprocessor.h>
#include <pointcloud_processor/region_creator.h>
#include <pointcloud_processor/parameter_determiner.h>
#include <pointcloud_processor/hull_finder.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;
using PlaneParameterVector = std::vector<pcl::ModelCoefficients::Ptr>;
using HullVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using PolygonVector = std::vector<std::vector<pcl::Vertices>>;

std::string POINTCLOUD_TOPIC = "/camera/depth/color/points";
ros::Duration POINTCLOUD_TIMEOUT = ros::Duration(1.0); // secs

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
  hull_finder_ = std::make_unique<SimpleHullFinder>(
      getConfigIfPresent("hull_finder"), debugging_);
  parameter_determiner_ = std::make_unique<SimpleParameterDeterminer>(
      getConfigIfPresent("parameter_determiner"), debugging_);


  if (debugging_) {
    ROS_DEBUG("Realsense reader started with debugging, all intermediate result "
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
    int selected_gait,
    march_shared_msgs::GetGaitParameters::Response &res)
{
  clock_t start_processing = clock();
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
    ROS_DEBUG("Done preprocessing, see /camera/preprocessed_cloud for results");
    publishPreprocessedPointCloud(pointcloud);
  }

  // Setup data structures for region creating
  boost::shared_ptr<RegionVector> region_vector =
      boost::make_shared<RegionVector>();
  // Create regions
  bool region_creating_was_successful =
      region_creator_->create_regions(pointcloud, normals, region_vector);
  if (not region_creating_was_successful)
  {
    res.error_message = "Region creating was unsuccessful, see debug output "
                        "for more information";
    return false;
  }

  ROS_DEBUG("Done creating regions");
  //TODO: Add publisher to visualize created regions

  // Setup data structures for hull finding
  boost::shared_ptr<PlaneParameterVector> plane_parameter_vector =
      boost::make_shared<PlaneParameterVector>();
  boost::shared_ptr<HullVector> hull_vector = boost::make_shared<HullVector>();
  boost::shared_ptr<PolygonVector> polygon_vector = boost::make_shared<PolygonVector>();
  // Find hulls
  bool hull_finding_was_successful =
      hull_finder_->find_hulls(pointcloud, normals, region_vector,
                                 plane_parameter_vector, hull_vector, polygon_vector);
  if (not hull_finding_was_successful)
  {
    res.error_message = "Hull finding was unsuccessful, see debug output "
                        "for more information";
    return false;
  }

  ROS_DEBUG("Done finding hulls");
  //TODO: Add publisher to visualize found planes

  // Setup data structures for parameter determining
  SelectedGait selected_obstacle = (SelectedGait) selected_gait;
  boost::shared_ptr<march_shared_msgs::GaitParameters> gait_parameters =
      boost::make_shared<march_shared_msgs::GaitParameters>();
  // Determine parameters
  bool parameter_determining_was_successful =
      parameter_determiner_->determine_parameters(
          plane_parameter_vector, hull_vector, polygon_vector, selected_obstacle,
          gait_parameters);
  if (not parameter_determining_was_successful)
  {
    res.error_message = "Parameter determining was unsuccessful, see debug output "
                        "for more information";
    return false;
  }
  res.gait_parameters = *gait_parameters;

  ROS_DEBUG("Done determining parameters");
  //TODO: Add publisher to visualize found hulls

  clock_t end_processing = clock();

  double time_taken = double(end_processing - start_processing) / double(CLOCKS_PER_SEC);
  ROS_DEBUG_STREAM("Time taken by point cloud processor is : " << std::fixed <<
                   time_taken << std::setprecision(5) << " sec " << std::endl);

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
  time_t start_callback = clock();

  boost::shared_ptr<const sensor_msgs::PointCloud2> input_cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>
      (POINTCLOUD_TOPIC, *n_, POINTCLOUD_TIMEOUT);

  if (input_cloud == nullptr)
  {
    res.error_message = "No pointcloud published within timeout, so "
                        "no processing could be done.";
    return false;
  }

  PointCloud converted_cloud;
  pcl::fromROSMsg(*input_cloud, converted_cloud);
  PointCloud::Ptr point_cloud = boost::make_shared<PointCloud>(converted_cloud);

  bool succes = process_pointcloud(point_cloud, req.selected_gait, res);

  time_t end_callback = clock();
  double time_taken = double(end_callback - start_callback) / double(CLOCKS_PER_SEC);
  ROS_DEBUG_STREAM("Time taken by process point cloud callback method: " << std::fixed <<
                   time_taken << std::setprecision(5) << " sec " << std::endl);

  return succes;
}