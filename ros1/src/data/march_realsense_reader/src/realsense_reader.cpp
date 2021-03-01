#include <march_realsense_reader/realsense_reader.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>
#include <pointcloud_processor/preprocessor.h>
#include <pointcloud_processor/region_creator.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;

RealSenseReader::RealSenseReader(ros::NodeHandle* n):
    n_(n),
    reading_(false)
{
  pointcloud_subscriber_ = n_->subscribe<PointCloud>
      ("/camera/depth/color/points", 1,
       &RealSenseReader::pointcloud_callback, this);
  read_pointcloud_service_ = n_->advertiseService
      ("/camera/read_pointcloud",
       &RealSenseReader::read_pointcloud_callback,
       this);
  config_tree_ = readConfig("pointcloud_parameters.yaml");

  preprocessor_ = std::make_unique<SimplePreprocessor>(
      getConfigIfPresent("preprocessor"));
  region_creator_ = std::make_unique<SimpleRegionCreator>(
      getConfigIfPresent("region_creator"));

  if (config_tree_["debug"])
  {
    debugging_ = config_tree_["debug"].as<bool>();
  } else
  {
    debugging_ = false;
  }

  if (debugging_)
  {
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
  } catch (YAML::Exception e)
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
  } else
  {
    ROS_WARN_STREAM("Key " << key << " was not found in the config file, empty config "
                                     "will be used");
    return YAML::Node();
  }
}

void RealSenseReader::pointcloud_callback(const PointCloud::ConstPtr& input_cloud)
{
  if (reading_)
  {
    // All logic to execute with a pointcloud will be executed here.
    reading_ = false;
    PointCloud::Ptr pointcloud = boost::make_shared<PointCloud>(*input_cloud);
    Normals::Ptr normals = boost::make_shared<Normals>();

    // Preprocess
    preprocessor_->preprocess(pointcloud, normals);
    if (debugging_)
    {
      preprocessed_pointcloud_publisher_.publish(pointcloud);
    }

    // Create regions
    boost::shared_ptr<RegionsVector> regions_vector =
        boost::make_shared<RegionsVector>();
    region_creator_->create_regions(pointcloud, normals, regions_vector);
  }
}

bool RealSenseReader::read_pointcloud_callback(std_srvs::Trigger::Request &req,
                                               std_srvs::Trigger::Response &res)
{
  reading_ = true;
  res.success = true;
  return true;
}