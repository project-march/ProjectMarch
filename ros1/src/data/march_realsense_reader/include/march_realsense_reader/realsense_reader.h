#ifndef MARCH_REALSENSE_READER_HPP
#define MARCH_REALSENSE_READER_HPP

#include <string>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <march_shared_msgs/GetGaitParameters.h>
#include "yaml-cpp/yaml.h"
#include <pointcloud_processor/preprocessor.h>
#include <pointcloud_processor/region_creator.h>
#include <pointcloud_processor/hull_finder.h>
#include <pointcloud_processor/parameter_determiner.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class RealSenseReader
{
public:
  // Setup realsense reader with given node handle.
  RealSenseReader(ros::NodeHandle* n);

  /** Takes the pointcloud and transform that into
   * gait parameters, these parameters are put into the response,
   * returns whether the processing was successful.
   */
  bool processPointcloud(PointCloud::Ptr input_cloud,
                          march_shared_msgs::GetGaitParameters::Response &res);

  /** A callback that starts the entire pointcloud processing when the
   * /camera/process_pointcloud service is called.
   */
  bool processPointcloudCallback(march_shared_msgs::GetGaitParameters::Request &req,
                                march_shared_msgs::GetGaitParameters::Response &res);

  /** Pointcloud callback, empty since we are not processing all pointclouds, this
   * gives a speedup when you need a single pointcloud.
   */
  void pointcloudCallback(const sensor_msgs::PointCloud2 pointCloud2) {};

  /** Read in the config file, the string is the name of the file, which should be
   * in the config directory. Returns a YAML::Node with the configurations.
   */
  YAML::Node readConfig(std::string config_file);

  // Get a config key from the root of the file, returns empty if key is missing.
  YAML::Node getConfigIfPresent(std::string key);

  // Publishes the pointcloud on a topic for visualisation in rviz or furter use
  template <typename T>
  void publishCloud(ros::Publisher publisher, pcl::PointCloud<T> cloud);

  // Turn a HullVector into a marker with a list of points and publish for visualization
  void publishHullMarkerArray(boost::shared_ptr<HullVector> hull_vector);

  // Create markers from the parameter determiner and publish them for visualization
  void publishParameterDeterminerMarkerArray();

  // Create a marker from the optimal foot location and publish it and publish for visualization
  void fillOptimalFootLocationMarker(pcl::PointNormal const optimal_foot_location,
                                     visualization_msgs::Marker & marker);

  // Create a marker list from the 'foot locations to try' and publish it and publish for visualization
  void fillFootLocationsToTryMarker(PointCloud2D::Ptr const foot_locations_to_try,
                                    visualization_msgs::Marker & marker_list);

  // Create a marker list from the 'foot locations to try' and publish it and publish for visualization
  void fillPossibleFootLocationsMarker(
          PointNormalCloud::Ptr const possible_foot_locations,
          pcl::PointNormal const optimal_foot_location,
          visualization_msgs::Marker & marker_list);

private:
  ros::NodeHandle* n_;
  ros::Subscriber pointcloud_subscriber_;
  PointCloud last_pointcloud_;
  ros::ServiceServer read_pointcloud_service_;
  ros::Publisher preprocessed_pointcloud_publisher_;
  ros::Publisher region_pointcloud_publisher_;
  ros::Publisher hull_marker_array_publisher_;
  ros::Publisher hull_parameter_determiner_publisher_;

  std::unique_ptr<NormalsPreprocessor> preprocessor_;
  std::unique_ptr<RegionGrower> region_creator_;
  std::unique_ptr<CHullFinder> hull_finder_;
  std::unique_ptr<HullParameterDeterminer> parameter_determiner_;
  bool debugging_;
  std::string config_file_;
  ros::Publisher pointcloud_publisher_;
  YAML::Node config_tree_;

  int selected_gait_;
  std::string frame_id_to_transform_to_;
  bool use_left_foot_;
};

#endif //MARCH_REALSENSE_READER_HPP
