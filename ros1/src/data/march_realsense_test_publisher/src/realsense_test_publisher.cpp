#include <filesystem>
#include <iostream>
#include <march_shared_msgs/PublishTestDataset.h>
#include <realsense_test_publisher.h>
#include <ros/package.h>
#include <string>
#include <utilities/camera_mode_utilities.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace std::filesystem;

std::string TOPIC_TEST_CLOUDS = "/test_clouds";
float PUBLISH_RATE = 1.0 / 5.0;

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

    publish_test_cloud_service = n_->advertiseService("/camera/publish_test_cloud",
        &RealsenseTestPublisher::publishTestDatasetCallback, this);

    test_cloud_publisher = n_->advertise<PointCloud>(TOPIC_TEST_CLOUDS, /*queue_size=*/1);
}

bool RealsenseTestPublisher::publishTestDatasetCallback(
    march_shared_msgs::PublishTestDataset::Request& req,
    march_shared_msgs::PublishTestDataset::Response& res)
{
    bool success = true;

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
        case SelectedMode::slide_show: {
//            publishSlideShow();
            ROS_DEBUG_STREAM("Now publishing a slide show of pointclouds");
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
    std::vector<path>::iterator path_iterator = std::find(file_paths.begin(), file_paths.end(), pointcloud_file_name);
    if (path_iterator == file_paths.end()) {
        std::string file_names_string = getFileNamesString();
        ROS_WARN_STREAM("The requested pointcloud file could not be found. Valid options are: \n" << file_names_string);
        return false;
    }

    PointCloud::Ptr pointcloud = boost::make_shared<PointCloud>();
    pcl::io::loadPLYFile<pcl::PointXYZ>((*path_iterator).path());
    ROS_DEBUG_STREAM("The file from path " << (*path_iterator).path() << "has been loaded up! now publishing")
    publishTestCloudOnTimer(pointcloud);
    return true;
}

void RealsenseTestPublisher::publishTestCloudOnTimer(PointCloud pointcloud)
{
    ros::Timer timer_publisher = n_.createTimer(ros::Duration(PUBLISH_RATE),
                                                std::bind(&publishTestCloud, pointcloud));
}

void RealsenseTestPublisher::publishTestCloud(PointCloud pointcloud)
{
    test_cloud_publisher.publish(pointcloud);
}

std::string RealsenseTestPublisher::getFileNamesString()
{
    std::string file_names_string;
    for (path path : file_paths) {
        file_names_string += path.filename().string() + "\n";
    }
    return file_names_string;
}
