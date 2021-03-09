#include <march_realsense_reader/realsense_reader.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>
#include <pointcloud_processor/preprocessor.h>
#include <pointcloud_processor/region_creator.h>
#include <ctime>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;

RealSenseReader::RealSenseReader(ros::NodeHandle* n):
    n_(n),
    reading_(false)
{
  pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>
      ("/camera/depth/color/points", 1,
       &RealSenseReader::pointcloud_callback, this);
  read_pointcloud_service_ = n_->advertiseService
      ("/camera/read_pointcloud",
       &RealSenseReader::read_pointcloud_callback,
       this);

  config_tree_ = readConfig("pointcloud_parameters.yaml");

  preprocessor_ = std::make_unique<NormalsPreprocessor>(
      getConfigIfPresent("preprocessor"));
  region_creator_ = std::make_unique<SimpleRegionCreator>(
      getConfigIfPresent("region_creator"));

  if (config_tree_["debug"])
  {
    debugging_ = config_tree_["debug"].as<bool>();
  }
  else
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


// When `reading_` is true, this method executes the logic to process a pointcloud
// on the next pointcloud the camera publishes. When `reading_` is false, this does nothing.
void RealSenseReader::pointcloud_callback(const sensor_msgs::PointCloud2 input_cloud)
{
  if (reading_)
  {
    clock_t start = clock();

    // All logic to execute with a pointcloud will be executed here.
    ROS_INFO_STREAM("Processing point cloud at time " << input_cloud.header.stamp);

    reading_ = false;

    clock_t start_convert = clock();
    PointCloud converted_cloud;
    pcl::fromROSMsg(input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud = boost::make_shared<PointCloud>(converted_cloud);
    Normals::Ptr normals = boost::make_shared<Normals>();
    clock_t end_convert = clock();

    // Preprocess
    preprocessor_->preprocess(pointcloud, normals);
    clock_t start_debug;
    clock_t end_debug;
    if (debugging_)
    {
      start_debug = clock();
      publishPreprocessedPointCloud(pointcloud);
      end_debug = clock();
    }
    // Create regions
    boost::shared_ptr<RegionsVector> regions_vector =
        boost::make_shared<RegionsVector>();
    region_creator_->create_regions(pointcloud, normals, regions_vector);

    clock_t end = clock();

    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by pointcloud_callback is : " << std::fixed
              << time_taken << std::setprecision(5);
    std::cout << " sec " << std::endl;
    time_taken = double(end_debug - start_debug) / double(CLOCKS_PER_SEC);
    std::cout << "Of which: " << std::fixed
              << time_taken << std::setprecision(5);
    std::cout << " sec is taken up by publising" << std::endl;
    time_taken = double(end_convert - start_convert) / double(CLOCKS_PER_SEC);
    std::cout << "And of which: " << std::fixed
              << time_taken << std::setprecision(5);
    std::cout << " sec is taken up by converting" << std::endl;
  }
}

// Sets the `reading_` variable to true so pointcloud_callback executes its logic
bool RealSenseReader::read_pointcloud_callback(std_srvs::Trigger::Request &req,
                                               std_srvs::Trigger::Response &res)
{
  reading_ = true;
  res.success = true;
  return true;
}

// Publishes the pointcloud on a topic for visualisation in rviz or furter use
void RealSenseReader::publishPreprocessedPointCloud(PointCloud::Ptr pointcloud)
{
  ROS_INFO_STREAM("Publishing a preprocessed cloud with size: " << pointcloud->points.size());

  pointcloud->width  = 1;
  pointcloud->height = pointcloud->points.size();

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*pointcloud, msg);

  preprocessed_pointcloud_publisher_.publish(msg);
}

