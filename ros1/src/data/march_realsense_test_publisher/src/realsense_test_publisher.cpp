#include <filesystem>
#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <realsense_test_publisher.h>
#include <ros/package.h>
#include <string>
#include <utilities/camera_mode_utilities.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::filesystem;

std::string TOPIC_TEST_CLOUDS = "/test_clouds";
float PUBLISH_RATE = 5.0; // images per second
std::string CAMERA_FRAME_ID = "camera_front_depth_optical_frame";

RealsenseTestPublisher::RealsenseTestPublisher(ros::NodeHandle* n)
    : n_(n)
{
    if (ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    path directory_path
        = ros::package::getPath("march_realsense_test_publisher");
    path relative_path("config/datasets/");
    data_path = directory_path / relative_path;

    for (const auto& entry : std::filesystem::directory_iterator(data_path)) {
        file_names.push_back(entry.path().filename().string());
    }

    publish_test_cloud_service
        = n_->advertiseService(/*service=*/"/camera/publish_test_cloud",
            &RealsenseTestPublisher::publishTestDatasetCallback, this);

    test_cloud_publisher
        = n_->advertise<PointCloud>(TOPIC_TEST_CLOUDS, /*queue_size=*/1);

    publishLoop();
}

bool RealsenseTestPublisher::publishTestDatasetCallback(
    march_shared_msgs::PublishTestDataset::Request& req,
    march_shared_msgs::PublishTestDataset::Response& res)
{
    selected_mode = (SelectedMode)req.selected_mode;
    pointcloud_file_name = req.pointcloud_file_name;
    return true;
}

bool RealsenseTestPublisher::publishCustomPointcloud(
    std::string pointcloud_file_name)
{
    std::vector<std::string>::iterator filename_iterator
        = std::find(file_names.begin(), file_names.end(), pointcloud_file_name);
    if (filename_iterator == file_names.end()) {
        ROS_WARN_STREAM("the given file name " << pointcloud_file_name << " is invalid. Must be one of " << getFileNamesString());
        return;
    }
    std::string selected_file_name = *filename_iterator;

    pointcloud_to_publish = boost::make_shared<PointCloud>();
    pcl::io::loadPLYFile<pcl::PointXYZ>(
        data_path.string() + selected_file_name, *pointcloud_to_publish);
    pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID;
    publishTestCloud();
}


void publishTestCloud()
{
    pcl_conversions::toPCL(ros::Time::now(), pointcloud_to_publish->header.stamp);
    test_cloud_publisher.publish(pointcloud_to_publish);
}

std::string RealsenseTestPublisher::getFileNamesString()
{
    std::string file_names_string;
    for (std::string name : file_names) {
        file_names_string += name + "\n";
    }
    return file_names_string;
}

void publishLoop()
{
    ros::Rate loop_rate(PUBLISH_RATE);
    while(n_->ok()) {
        switch (selected_mode) {
            case SelectedMode::start: {
                //            startPublishingPointclouds();
                ROS_DEBUG_STREAM_THROTTLE("Started publishing pointclouds");
                break;
            }
            case SelectedMode::next: {
                //            publishNextPointcloud();
                ROS_DEBUG_STREAM("now publishing next pointcloud");
                break;
            }
            case SelectedMode::custom: {
                publishCustomPointcloud(pointcloud_file_name);
                break;
            }
            case SelectedMode::slide_show: {
                //            publishSlideShow();
                ROS_DEBUG_STREAM("Now publishing a slide show of pointclouds");
            }
            case SelectedMode::end: {
                ROS_DEBUG_STREAM("Stopped publishing pointclouds");
                break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}