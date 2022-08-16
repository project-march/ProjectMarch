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
#include <utilities/yaml_utilities.h>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::filesystem;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class RealsenseTestPublisher {
public:
    // Setup realsense reader with given node handle.
    explicit RealsenseTestPublisher(ros::NodeHandle* n);

    // Publish the current pointcloud to publish
    // is public to allow a timer call from the node
    void publishTestCloud(const ros::TimerEvent& timer_event);

private:
    // Creates a string of all the valid file names separated by an end line
    std::string getFileNamesString();

    // Sets the new publish mode, the camera position in which the pointcloud
    // has been created, the arguments for the process call and, if relevant,
    // the requested pointcloud file name
    bool publishTestDatasetCallback(
        march_shared_msgs::PublishTestDataset::Request& req, march_shared_msgs::PublishTestDataset::Response& res);

    // Sets the right cloud as the pointcloud to publish based on the file name
    bool loadPointcloudToPublishFromFilename();

    // Publishes the first pointcloud in the dataset directory
    bool startPublishingPointclouds();

    // Publishes the next pointcloud in the dataset directory
    bool publishNextPointcloud();

    // Publish the right pointcloud based on the latest service call
    void updatePublishLoop(march_shared_msgs::PublishTestDataset::Response& res);

    // flips the sign of the z coordinates of the cloud, necessary because of a
    // weird inconsistency between the coordinate systems in the realsense
    // viewer and the .ply files
    void transformToCameraCoordinates();

    // Grabs the inputs needed for the process pointcloud service call from the
    // configuration file
    void getProcessPointcloudInputs();

    // Load the configuration yaml file
    YAML::Node loadConfig(const std::string& config_file);

    // Calls on the realsense reader to process a pointcloud from the test topic
    // is public to allow a timer call from the node
    void makeProcessPointcloudCall();

    // Save the current pointcloud being published on either the front or back
    // camera topic as a .ply file to test with later
    bool saveCurrentPointcloud();

    // Update the currently available file names
    void updateFileNamesVector();

    ros::NodeHandle* n_;
    ros::ServiceServer publish_test_cloud_service;
    ros::Publisher test_cloud_publisher;
    ros::ServiceClient process_pointcloud_service_client;
    YAML::Node config_tree;

    std::vector<std::string> file_names;
    path data_path;
    path write_path;
    PointCloud::Ptr pointcloud_to_publish;
    std::string pointcloud_file_name;
    std::string subgait_name;
    std::string save_pointcloud_name;
    std::string pointcloud_topic;
    bool should_publish;
    bool from_back_camera;
    bool from_realsense_viewer;
    bool save_camera_back;
    SelectedMode selected_mode;
    int realsense_category;
};

#endif // MARCH_REALSENSE_TEST_PUBLISHER_H
