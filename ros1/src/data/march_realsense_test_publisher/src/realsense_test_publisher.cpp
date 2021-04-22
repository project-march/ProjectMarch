#include <filesystem>
#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <utilities/camera_mode_utilities.h>
#include <realsense_test_publisher.h>
#include <ros/package.h>
#include <string>

using namespace std::filesystem;

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
    SelectedMode selected_mode = (SelectedMode) req.selected_mode;
    switch (selected_mode) {
        case SelectedMode::start: {
            printPointcloudNames();
            break;
        }
        case SelectedMode::end: {
            ROS_WARN_STREAM("TESTING, YOU PRESSED END");
            break;
        }
    }
    res.success = true;
    return true;
}

void RealsenseTestPublisher::printPointcloudNames()
{
    for (std::string path : file_paths) {
        ROS_DEBUG_STREAM(path);
    }
}
