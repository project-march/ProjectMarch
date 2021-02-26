#ifndef MARCH_REALSENSE_READER_HPP
#define MARCH_REALSENSE_READER_HPP

#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class RealSenseReader
{
public:
    RealSenseReader(ros::NodeHandle* n);
    void pointcloud_callback(const PointCloud::ConstPtr& input_cloud);
    bool read_pointcloud_callback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);
    YAML::Node readConfig(std::string config_file);
    YAML::Node getConfigIfPresent(std::string key);

private:
    ros::NodeHandle* n_;
    ros::Subscriber pointcloud_subscriber_;
    ros::ServiceServer read_pointcloud_service_;
    ros::Publisher preprocessed_pointcloud_publisher_;
    bool reading_;
    bool debugging_;
    YAML::Node config_tree_;
};

#endif //MARCH_REALSENSE_READER_HPP
