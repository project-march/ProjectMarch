#ifndef MARCH_REALSENSE_TEST_PUBLISHER_H
#define MARCH_REALSENSE_TEST_PUBLISHER_H

#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <filesystem>

using namespace std::filesystem;
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

    // Publishes the first pointcloud in the dataset directory
    void startPublishingPointclouds();

    // Publishes the next pointcloud in the statset directory
    void publishNextPointcloud();

    // Publishes the pointcloud with the requested file name
    bool publishCustomPointcloud(std::string pointcloud_file_name);

    // Stops publishing pointclouds
    void stopPublishingPointClouds();

    ros::NodeHandle* n_;
    ros::ServiceServer publish_service_;

    std::vector<path> file_paths;
    std::string pointcloud_topic;
};

#endif // MARCH_REALSENSE_TEST_PUBLISHER_H
