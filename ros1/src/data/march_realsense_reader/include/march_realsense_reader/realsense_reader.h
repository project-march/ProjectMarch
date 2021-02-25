#ifndef MARCH_REALSENSE_READER_HPP
#define MARCH_REALSENSE_READER_HPP

#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class RealSenseReader
{
public:
    RealSenseReader(ros::NodeHandle* n);
    void pointcloud_callback(const sensor_msgs::PointCloud2 input_cloud);
    bool read_pointcloud_callback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);

private:
    ros::NodeHandle* n_;
    ros::Subscriber pointcloud_subscriber_;
    PointCloud last_pointcloud_;
    ros::ServiceServer read_pointcloud_service_;
    bool reading_;
    std::string config_file_;
    ros::Publisher pointcloud_publisher_;
};

#endif //MARCH_REALSENSE_READER_HPP
