#include <filesystem>
#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <realsense_test_publisher.h>
#include <string>
#include <ros/package.h>

using namespace std::filesystem;

RealsenseTestPublisher::RealsenseTestPublisher(ros::NodeHandle* n)
    : n_(n)
{
    if (ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    path directory_path = ros::package::getPath("march_realsense_test_publisher");
    ROS_WARN_STREAM(directory_path.u8string());

    path relative_path("config/data_sets/");
    path data_path = directory_path / relative_path;

    for (const auto& entry : std::filesystem::directory_iterator(data_path)) {
        file_paths.push_back(entry.path());
    }

    publish_service_ = n_->advertiseService("/camera/publish_tests",
        &RealsenseTestPublisher::publishTestDatasetCallback, this);
}

bool RealsenseTestPublisher::publishTestDatasetCallback(
    march_shared_msgs::PublishTestDataset::Request& req,
    march_shared_msgs::PublishTestDataset::Response& res)
{
    printPointcloudNames();
    return true;
}

void RealsenseTestPublisher::printPointcloudNames()
{
    for (std::string path : file_paths) {
        ROS_DEBUG_STREAM(path);
    }
}
