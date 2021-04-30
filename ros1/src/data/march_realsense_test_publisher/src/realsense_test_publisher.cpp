#include <filesystem>
#include <iostream>
#include <march_shared_msgs/GetGaitParameters.h>
#include <march_shared_msgs/PublishTestDataset.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <realsense_test_publisher.h>
#include <ros/package.h>
#include <string>
#include <utilities/publish_mode_utilities.h>
#include <utilities/realsense_gait_utilities.h>

using namespace std::filesystem;

std::string TOPIC_TEST_CLOUDS = "/test_clouds";
std::string CAMERA_FRAME_ID_FRONT = "camera_front_depth_optical_frame";
std::string CAMERA_FRAME_ID_BACK = "camera_back_depth_optimal_frame";
std::string PROCESS_POINTCLOUD_TOPIC = "/camera/process_pointcloud";
std::string POINTCLOUD_EXTENSION = ".ply";

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
        if (std::filesystem::is_regular_file(entry)
            && entry.path().extension() == POINTCLOUD_EXTENSION) {
            file_names.push_back(entry.path().filename().string());
        }
    }
    no_files_present = (file_names.size() == 0);
    if (no_files_present) {
        ROS_ERROR_STREAM("There are no .ply files present under path "
            << data_path << ". Test publisher will not work.");
    }
    publish_test_cloud_service
        = n_->advertiseService(/*service=*/"/camera/publish_test_cloud",
            &RealsenseTestPublisher::publishTestDatasetCallback, this);

    test_cloud_publisher
        = n_->advertise<PointCloud>(TOPIC_TEST_CLOUDS, /*queue_size=*/1);

    process_pointcloud_service_client
        = n_->serviceClient<march_shared_msgs::GetGaitParameters>(
            PROCESS_POINTCLOUD_TOPIC);

    should_publish = false;
}

// Sets the new publish mode, the camera position in which the pointcloud has
// been created, the arguments for the process call and, if relevant, the
// requested pointcloud file name
bool RealsenseTestPublisher::publishTestDatasetCallback(
    march_shared_msgs::PublishTestDataset::Request& req,
    march_shared_msgs::PublishTestDataset::Response& res)
{
    selected_mode = (SelectedMode)req.selected_mode;
    selected_gait = (SelectedGait)req.selected_gait;
    frame_id_to_transform_to = req.frame_id_to_transform_to;

    from_back_camera = req.from_back_camera;
    // Only update the pointcloud file name from the service if it is relevant
    if (selected_mode == SelectedMode::custom) {
        pointcloud_file_name = req.pointcloud_file_name;
    }
    updatePublishLoop(res);
    return true;
}

// Sets the right cloud as the pointcloud to publish based on the file name
void RealsenseTestPublisher::loadPointcloudToPublishFromFilename()
{
    mirrorZCoordinate();
    pointcloud_to_publish = boost::make_shared<PointCloud>();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(
            data_path.string() + pointcloud_file_name, *pointcloud_to_publish)
        == -1) {
        ROS_WARN_STREAM("Couldn't find file from path "
            << data_path.string() + pointcloud_file_name);
    } else {
        ROS_DEBUG_STREAM("File loaded.");
    }
}

// Publishes the pointcloud with the requested file name
bool RealsenseTestPublisher::publishCustomPointcloud(
    std::string pointcloud_file_name)
{
    std::vector<std::string>::iterator filename_iterator
        = std::find(file_names.begin(), file_names.end(), pointcloud_file_name);
    if (filename_iterator == file_names.end()) {
        ROS_WARN_STREAM("the given file name " << pointcloud_file_name
                                               << " is invalid. Must be one of "
                                               << getFileNamesString());
        return false;
    }

    loadPointcloudToPublishFromFilename();
    return true;
}

// Publish the current pointcloud to publish
void RealsenseTestPublisher::publishTestCloud(
    const ros::TimerEvent& timer_event)
{
    if (should_publish) {
        if (from_back_camera) {
            pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID_BACK;
        } else {
            pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID_FRONT;
        }
        pcl_conversions::toPCL(
            ros::Time::now(), pointcloud_to_publish->header.stamp);
        test_cloud_publisher.publish(pointcloud_to_publish);
    }
}

// Creates a string of all the valid file names separated by an end line
std::string RealsenseTestPublisher::getFileNamesString()
{
    // Start with an end line for ease of printing
    std::string file_names_string = "\n";
    for (std::string name : file_names) {
        file_names_string += name + "\n";
    }
    return file_names_string;
}

// Publishes the first pointcloud in the dataset directory
void RealsenseTestPublisher::startPublishingPointclouds()
{
    pointcloud_file_name = file_names[0];
    loadPointcloudToPublishFromFilename();
}

// Publishes the next pointcloud in the dataset directory
void RealsenseTestPublisher::publishNextPointcloud()
{
    // If already publishing, find the next pointcloud and publish that
    // Otherwise, start publishing
    if (should_publish) {
        // find the current pointcloud filename
        std::vector<std::string>::iterator filename_iterator = std::find(
            file_names.begin(), file_names.end(), pointcloud_file_name);

        // Set the current pointcloud file name to the next name in the list, if
        // the old name is the last in the list, set the new name to the first
        // in the list
        if (filename_iterator == file_names.end()) {
            ROS_WARN_STREAM(
                "The old pointcloud file name could not be found in "
                "the file name vector. Unable to find next pointcloud.");
        } else if (filename_iterator == file_names.end() - 1) {
            pointcloud_file_name = file_names[0];
        } else {
            pointcloud_file_name = *(filename_iterator + 1);
        }
        loadPointcloudToPublishFromFilename();
    } else {
        startPublishingPointclouds();
    }
}

// Publish the right pointcloud based on the latest service call
void RealsenseTestPublisher::updatePublishLoop(
    march_shared_msgs::PublishTestDataset::Response& res)
{
    // Only update the publish loop if there are files available
    if (!no_files_present) {
        // The update is successful by default until something goes wrong
        bool success = true;

        switch (selected_mode) {
            case SelectedMode::start: {
                ROS_DEBUG_STREAM("Start publishing pointclouds");
                startPublishingPointclouds();
                should_publish = true;
                break;
            }
            case SelectedMode::next: {
                ROS_DEBUG_STREAM("Publish next pointcloud");
                publishNextPointcloud();
                should_publish = true;
                break;
            }
            case SelectedMode::custom: {
                ROS_DEBUG_STREAM("Publish a custom pointcloud");
                success &= publishCustomPointcloud(pointcloud_file_name);
                should_publish = success;
                break;
            }
            case SelectedMode::end: {
                ROS_DEBUG_STREAM("Stop publishing pointclouds");
                should_publish = false;
                break;
            }
            default: {
                ROS_WARN_STREAM("Invalid mode selected");
                return;
            }
        }
        if (selected_mode != SelectedMode::end) {
            makeProcessPointcloudCall();
        }
        res.success = success;
    } else {
        ROS_ERROR_STREAM(
            "No .ply files can be found by the test publisher under path "
            << data_path << ". Unable to publish a test cloud.");
    }
}

// flips the sign of the z coordinates of the cloud, necessary because of a
// weird inconsistency between the coordinate systems in the realsense
// viewer and the .ply files
void RealsenseTestPublisher::mirrorZCoordinate()
{
    for (size_t i = 0; i < pointcloud_to_publish->size(); ++i) {
        float z_value = pointcloud_to_publish->points[i].z;
        pointcloud_to_publish->points[i].z = -z_value;
    }
}

// Calls on the realsense reader to process a pointcloud from the test topic
void RealsenseTestPublisher::makeProcessPointcloudCall()
{
    march_shared_msgs::GetGaitParameters service;
    service.request.selected_gait = 0;
    service.request.frame_id_to_transform_to = "foot_left";
    service.request.camera_to_use = 2;
    process_pointcloud_service_client.call(service);
}
