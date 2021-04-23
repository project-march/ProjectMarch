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
    // Creates a string of all the valid file names separated by an end line
    std::string getFileNamesString();

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

    // Publish a pointcloud on a timer
    void publishTestCloudOnTimer();

    // Publish a pointcloud
    void publishTestCloud();

    ros::NodeHandle* n_;
    ros::ServiceServer publish_test_cloud_service;
    ros::Publisher test_cloud_publisher;

    std::vector<path> file_paths;
    std::string pointcloud_topic;
    PointCloud::Ptr pointcloud_to_publish;
};

#endif // MARCH_REALSENSE_TEST_PUBLISHER_H
