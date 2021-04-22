#include <filesystem>
#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <realsense_test_publisher.h>
#include <ros/package.h>
#include <string>
#include <utilities/camera_mode_utilities.h>

using namespace std::filesystem;

std::string POINTCLOUD_FRONT_TOPIC = "/camera_front/depth/color/points";
std::string POINTCLOUD_BACK_TOPIC = "/camera_back/depth/color/points";

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
    bool success = true;

    if (req.use_front_camera) {
        pointcloud_topic = POINTCLOUD_FRONT_TOPIC;
    } else {
        pointcloud_topic = POINTCLOUD_BACK_TOPIC;
    }

    SelectedMode selected_mode = (SelectedMode)req.selected_mode;
    switch (selected_mode) {
        case SelectedMode::start: {
//            startPublishingPointclouds();
            ROS_DEBUG_STREAM("Started publishing pointclouds");
            break;
        }
        case SelectedMode::next: {
//            publishNextPointcloud();
            ROS_DEBUG_STREAM("now publishing next pointcloud");
            break;
        }
        case SelectedMode::custom: {
            success &= publishCustomPointcloud(req.pointcloud_file_name);
            if (success) {
                ROS_DEBUG_STREAM("Now publishing pointcloud with file name "
                    << req.pointcloud_file_name);
            } else {
                ROS_DEBUG_STREAM("Failed to publish pointcloud with file name "
                    << req.pointcloud_file_name);
            }
            break;
        }
        case SelectedMode::end: {
//            stopPublishingPointClouds();
            ROS_DEBUG_STREAM("Stopped publishing pointclouds");
            break;
        }
    }
    res.success = success;
    return success;
}

bool RealsenseTestPublisher::publishCustomPointcloud(std::string pointcloud_file_name)
{
    printPointcloudNames();
    return false;
}

void RealsenseTestPublisher::printPointcloudNames()
{
    for (path path : file_paths) {
        ROS_DEBUG_STREAM(path.filename().string());
    }
}
