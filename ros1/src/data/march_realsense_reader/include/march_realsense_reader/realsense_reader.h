#ifndef MARCH_REALSENSE_READER_HPP
#define MARCH_REALSENSE_READER_HPP

#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class RealSenseReader
{
public:
    RealSenseReader(ros::NodeHandle* n);
    void pointcloud_callback(const PointCloud::ConstPtr& msg);

private:
    ros::NodeHandle* n_;
    ros::Subscriber pointcloud_subscriber_;
};

#endif //MARCH_REALSENSE_READER_HPP
