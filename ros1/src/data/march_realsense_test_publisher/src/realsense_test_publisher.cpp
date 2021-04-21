#include <filesystem>
#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <realsense_test_publisher.h>
#include <string>

using std::filesystem::current_path;

RealsenseTestPublisher::RealsenseTestPublisher(ros::NodeHandle* n)
    : n_(n)
{
    if (ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    char tmp[256];
    ROS_WARN_STREAM(getcwd(tmp, 256));

    std::string path = "config/data_sets/";
    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        file_paths.push_back(entry.path());
    }

    publish_service_ = n_->advertiseService("/camera/publish_tests",
        &RealsenseTestPublisher::publishTestDatasetCallback, this);
}

bool RealsenseTestPublisher::publishTestDatasetCallback(
    march_shared_msgs::PublishTestDataset::Request& req,
    march_shared_msgs::PublishTestDataset::Response& res)
{
    ROS_DEBUG_STREAM(req.selected_mode);
    ROS_DEBUG_STREAM(req.selected_camera);
    printPointcloudNames();
    return true;
}

void RealsenseTestPublisher::printPointcloudNames()
{
    for (std::string path : file_paths) {
        ROS_DEBUG_STREAM(path);
    }
}
