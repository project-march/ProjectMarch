#ifndef MARCH_REALSENSE_TEST_PUBLISHER_H
#define MARCH_REALSENSE_TEST_PUBLISHER_H

#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>
#include <vector>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class RealsenseTestPublisher {
public:
    // Setup realsense reader with given node handle.
    RealsenseTestPublisher(ros::NodeHandle* n);

private:
    // Outputs the name of all available pointcloud files to debug
    void printPointcloudNames();

    // Publishes the right pointcloud on the right topic
    bool publishTestDatasetCallback(
        march_shared_msgs::PublishTestDataset::Request& req,
        march_shared_msgs::PublishTestDataset::Response& res);

    ros::NodeHandle* n_;
    ros::ServiceServer publish_service_;

    std::vector<std::string> file_paths;
};

#endif // MARCH_REALSENSE_TEST_PUBLISHER_H
