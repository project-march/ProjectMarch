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

    testForParseErrors();
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
                std::string file_names_string = getFileNamesString();

                ROS_WARN_STREAM("Failed to publish pointcloud with file name "
                    << req.pointcloud_file_name << " \n Valid options are: \n"
                    << file_names_string);
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

bool RealsenseTestPublisher::publishCustomPointcloud(
    std::string pointcloud_file_name)
{
    std::vector<std::string>::iterator filename_iterator
        = std::find(file_names.begin(), file_names.end(), pointcloud_file_name);
    if (filename_iterator == file_names.end()) {
        return false;
    }
    std::string selected_file_name = *filename_iterator;

    pointcloud_to_publish = boost::make_shared<PointCloud>();
    pcl::io::loadPLYFile<pcl::PointXYZ>(
        data_path.string() + selected_file_name, *pointcloud_to_publish);
    ROS_DEBUG_STREAM("The file from path "
        << selected_file_name << " has been loaded up! now publishing a pointcloud with "
        << pointcloud_to_publish->points.size() << " points.");
    publishTestCloudOnTimer();
    return true;
}

void RealsenseTestPublisher::publishTestCloudOnTimer()
{
//    ros::Timer timer_publisher = n_->createTimer(ros::Duration(PUBLISH_RATE),
//        std::bind(&RealsenseTestPublisher::publishTestCloud, this));
//    ROS_WARN_STREAM("The timer has been created");

    ros::Rate loop_rate(PUBLISH_RATE);
    pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID;
    while (n_->ok()) {
        pcl_conversions::toPCL(ros::Time::now(), pointcloud_to_publish->header.stamp);
        test_cloud_publisher.publish(pointcloud_to_publish);
        ROS_WARN_STREAM("Pointcloud has actually been published");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//void RealsenseTestPublisher::publishTestCloud()
//{
//    ROS_WARN_STREAM("The timer calls on the publisher");
//    test_cloud_publisher.publish(pointcloud_to_publish);
//}

std::string RealsenseTestPublisher::getFileNamesString()
{
    std::string file_names_string;
    for (std::string name : file_names) {
        file_names_string += name + "\n";
    }
    return file_names_string;
}

// This is temporary
void RealsenseTestPublisher::testForParseErrors()
{
//    for (std::string name : file_names) {
//        pointcloud_to_publish = boost::make_shared<PointCloud>();
//        pcl::io::loadPLYFile<pcl::PointXYZ>(
//                data_path.string() + name, *pointcloud_to_publish);
//        ROS_DEBUG_STREAM("The file from path "
//            << name << " has been loaded up! now publishing a pointcloud with "
//            << pointcloud_to_publish->points.size() << " points.");
//        publishTestCloudOnTimer();
//    }
}
