#ifndef MARCH_REALSENSE_TEST_PUBLISHER_H
#define MARCH_REALSENSE_TEST_PUBLISHER_H

#include <filesystem>
#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>
#include <utilities/publish_mode_utilities.h>
#include <utilities/realsense_gait_utilities.h>
#include <vector>

using namespace std::filesystem;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class RealsenseTestPublisher {
public:
    // Setup realsense reader with given node handle.
    RealsenseTestPublisher(ros::NodeHandle* n);

    // Publish the current pointcloud to publish
    // is public to allow a timer call from the node
    void publishTestCloud(const ros::TimerEvent& timer_event);

    // Calls on the realsense reader to process a pointcloud from the test topic
    // is public to allow a timer call from the node
    void makeProcessPointcloudCall(const ros::TimerEvent& timer_event);

private:
    // Creates a string of all the valid file names separated by an end line
    std::string getFileNamesString();

    // Sets the new publish mode, the camera position in which the pointcloud
    // has been created, the arguments for the process call and, if relevant,
    // the requested pointcloud file name
    bool publishTestDatasetCallback(
        march_shared_msgs::PublishTestDataset::Request& req,
        march_shared_msgs::PublishTestDataset::Response& res);

    // Sets the right cloud as the pointcloud to publish based on the file name
    void loadPointcloudToPublishFromFilename();

    // Publishes the first pointcloud in the dataset directory
    void startPublishingPointclouds();

    // Publishes the next pointcloud in the dataset directory
    void publishNextPointcloud();

    // Publishes the pointcloud with the requested file name
    bool publishCustomPointcloud(std::string pointcloud_file_name);

    // Publish the right pointcloud based on the latest service call
    void updatePublishLoop(
        march_shared_msgs::PublishTestDataset::Response& res);

    // flips the sign of the z coordinates of the cloud, necessary because of a
    // weird inconsistency between the coordinate systems in the realsense
    // viewer and the .ply files
    void mirrorZCoordinate();

    ros::NodeHandle* n_;
    ros::ServiceServer publish_test_cloud_service;
    ros::Publisher test_cloud_publisher;
    ros::ServiceClient process_pointcloud_service_client;

    std::vector<std::string> file_names;
    path data_path;
    std::string pointcloud_topic;
    PointCloud::Ptr pointcloud_to_publish;
    std::string pointcloud_file_name;
    SelectedMode selected_mode;
    bool should_publish;
    bool no_files_present;
    bool from_back_camera;
    bool should_process;
    SelectedGait selected_gait;
    std::string frame_id_to_transform_to;
};

#endif // MARCH_REALSENSE_TEST_PUBLISHER_H
