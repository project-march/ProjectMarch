#include <filesystem>
#include <iostream>
#include <march_shared_msgs/GetGaitParameters.h>
#include <march_shared_msgs/PublishTestDataset.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <realsense_test_publisher.h>
#include <ros/package.h>
#include <string>
#include <utilities/publish_mode_utilities.h>

using namespace std::filesystem;

std::string TOPIC_CAMERA_FRONT = "/camera_front/depth/color/points";
std::string TOPIC_CAMERA_BACK = "/camera_back/depth/color/points";
std::string TOPIC_TEST_CLOUDS = "/test_clouds";
std::string CAMERA_FRAME_ID_FRONT = "camera_front_depth_optical_frame";
std::string CAMERA_FRAME_ID_BACK = "camera_back_depth_optimal_frame";
std::string PROCESS_POINTCLOUD_SERVICE_NAME = "/camera/process_pointcloud";
std::string PUBLISH_POINTCLOUD_SERVICE_NAME = "/camera/publish_pointcloud";
std::string DATASET_CONFIGURATION_NAME = "pointcloud_information.yaml";
std::string POINTCLOUD_EXTENSION = ".ply";
ros::Duration POINTCLOUD_TIMEOUT = ros::Duration(/*t=*/1.0); // secs

RealsenseTestPublisher::RealsenseTestPublisher(ros::NodeHandle* n)
    : n_(n)
    , should_publish(false)
    , from_back_camera(false)
    , selected_mode((SelectedMode)-1)
    , realsense_category(-1)
{
    if (ros::console::set_logger_level(
            ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    path directory_path
        = ros::package::getPath("march_realsense_test_publisher");
    path relative_path(/*__source*/ "config/datasets/");
    data_path = directory_path / relative_path;

    for (const auto& entry : std::filesystem::directory_iterator(data_path)) {
        if (std::filesystem::is_regular_file(entry)
            && entry.path().extension() == POINTCLOUD_EXTENSION) {
            file_names.push_back(entry.path().filename().string());
        }
    }
    if (file_names.size() == 0) {
        ROS_ERROR_STREAM("There are no .ply files present under path "
            << data_path << ". Shutting down the test publisher.");
        ros::shutdown();
    }
    publish_test_cloud_service
        = n_->advertiseService(/*service=*/"/camera/publish_test_cloud",
            &RealsenseTestPublisher::publishTestDatasetCallback, this);

    test_cloud_publisher
        = n_->advertise<PointCloud>(TOPIC_TEST_CLOUDS, /*queue_size=*/1);

    process_pointcloud_service_client
        = n_->serviceClient<march_shared_msgs::GetGaitParameters>(
            PROCESS_POINTCLOUD_SERVICE_NAME);

    config_tree = loadConfig(DATASET_CONFIGURATION_NAME);
}

YAML::Node RealsenseTestPublisher::loadConfig(const std::string& config_file)
{
    YAML::Node config_tree;
    std::string path = ros::package::getPath("march_realsense_test_publisher")
        + "/config/" + config_file;
    try {
        config_tree = YAML::LoadFile(path);
    } catch (YAML::Exception& e) {
        ROS_WARN_STREAM("YAML file with path " << path
                                               << " could not be loaded, using "
                                                  "empty config instead");
    }
    return config_tree;
}

// Sets the new publish mode, the camera position in which the pointcloud has
// been created, the arguments for the process call and, if relevant, the
// requested pointcloud file name
bool RealsenseTestPublisher::publishTestDatasetCallback(
    march_shared_msgs::PublishTestDataset::Request& req,
    march_shared_msgs::PublishTestDataset::Response& res)
{
    selected_mode = (SelectedMode)req.selected_mode;
    // Only update the pointcloud file name from the service if it is relevant
    if (selected_mode == SelectedMode::custom) {
        pointcloud_file_name = req.pointcloud_file_name;
    }
    if (selected_mode == SelectedMode::Save) {
        save_pointcloud_name = req.pointcloud_file_name;
        save_camera_back = req.save_camera_back;
    }
    updatePublishLoop(res);
    return true;
}

void RealsenseTestPublisher::getProcessPointcloudInputs()
{
    if (YAML::Node pointcloud_config = config_tree[pointcloud_file_name]) {
        realsense_category = yaml_utilities::grabParameter<int>(
            pointcloud_config, "realsense_category")
                                 .value();
        from_back_camera = yaml_utilities::grabParameter<bool>(
            pointcloud_config, "from_back_camera")
                               .value();
        frame_id_to_transform_to = yaml_utilities::grabParameter<std::string>(
            pointcloud_config, "frame_id_to_transform_to")
                                       .value();
    } else {
        ROS_WARN_STREAM(
            "No configuration specified for pointcloud file with name "
            << pointcloud_file_name << ". Continuing with default parameters");
        realsense_category = 0;
        from_back_camera = false;
        frame_id_to_transform_to = "foot_right";
    }
}

// Sets the right cloud as the pointcloud to publish based on the file name
void RealsenseTestPublisher::loadPointcloudToPublishFromFilename()
{
    getProcessPointcloudInputs();

    pointcloud_to_publish = boost::make_shared<PointCloud>();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(
            data_path.string() + pointcloud_file_name, *pointcloud_to_publish)
        == -1) {
        ROS_WARN_STREAM("Couldn't find file from path "
            << data_path.string() + pointcloud_file_name);
        return;
    }

    if (from_back_camera) {
        pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID_BACK;
    } else {
        pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID_FRONT;
    }
    transformToCameraCoordinates();

    ROS_DEBUG_STREAM("File loaded with name " << pointcloud_file_name << ".");
}

// Publishes the pointcloud with the requested file name
bool RealsenseTestPublisher::publishCustomPointcloud(
    const std::string& pointcloud_file_name)
{
    ROS_DEBUG_STREAM("Publish a custom pointcloud");
    auto filename_iterator
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
    for (const std::string& name : file_names) {
        file_names_string += name + "\n";
    }
    return file_names_string;
}

// Publishes the first pointcloud in the dataset directory
void RealsenseTestPublisher::startPublishingPointclouds()
{
    ROS_DEBUG_STREAM("Start publishing pointcloud");
    // Start at a random index to reduce over fitting on one data set
    int random_index = rand() % file_names.size(); // NOLINT
    pointcloud_file_name = file_names[random_index];
    loadPointcloudToPublishFromFilename();
}

// Publishes the next pointcloud in the dataset directory
void RealsenseTestPublisher::publishNextPointcloud()
{
    // If already publishing, find the next pointcloud and publish that
    // Otherwise, start publishing
    if (should_publish) {
        ROS_DEBUG_STREAM("Publish next pointcloud");
        // find the current pointcloud filename
        auto filename_iterator = std::find(
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

bool RealsenseTestPublisher::saveCurrentPointcloud()
{
    // Set the correct pointcloud topic
    std::string pointcloud_topic;
    if (use_camera_back) {
        pointcloud_topic = TOPIC_CAMERA_BACK;
    } else {
        poitncloud_topic = TOPIC_CAMERA_FRONT;
    }
    // get the next pointcloud from the topic
    boost::shared_ptr<const sensor_msgs::PointCloud2> input_cloud
        = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            POINTCLOUD_TOPICS[req.camera_to_use], *n_, POINTCLOUD_TIMEOUT);

    if (input_cloud == nullptr) {
        res.error_message = ;
        ROS_WARN_STREAM("No pointcloud published within timeout on topic "
            << pointcloud_topic
            << ", so "
               "no saving could be done.");
        res.success = false;
    }
    PointCloud converted_cloud;
    pcl::fromROSMsg(*input_cloud, converted_cloud);
    PointCloud::Ptr point_cloud
        = boost::make_shared<PointCloud>(converted_cloud);
}

// Publish the right pointcloud based on the latest service call
void RealsenseTestPublisher::updatePublishLoop(
    march_shared_msgs::PublishTestDataset::Response& res)
{
    // The update is successful by default until something goes wrong
    bool success = true;

    switch (selected_mode) {
        case SelectedMode::next: {
            publishNextPointcloud();
            should_publish = true;
            break;
        }
        case SelectedMode::custom: {
            success &= publishCustomPointcloud(pointcloud_file_name);
            should_publish = success;
            break;
        }
        case SelectedMode::end: {
            ROS_DEBUG_STREAM("Stop publishing pointclouds");
            should_publish = false;
            break;
        }
        case SelectedMode::Save: {
            ROS_DEBUG_STREAM("Save the current pointcloud")
            saveCurrentpointcloud();
            break;
        }
        default: {
            ROS_WARN_STREAM("Invalid mode selected");
            return;
        }
    }
    if (selected_mode != SelectedMode::end) {
        makeProcessPointcloudCall();
        res.message
            = "Now publishing pointcloud with name " + pointcloud_file_name;
    }
    res.success = success;
}

// flips the sign of the z and y coordinates of the cloud, necessary because of
// a weird inconsistency between the coordinate systems in the realsense viewer
// and the .ply files
void RealsenseTestPublisher::transformToCameraCoordinates()
{
    for (size_t i = 0; i < pointcloud_to_publish->size(); ++i) {
        float z_value = pointcloud_to_publish->points[i].z;
        float y_value = pointcloud_to_publish->points[i].y;
        pointcloud_to_publish->points[i].z = -z_value;
        pointcloud_to_publish->points[i].y = -y_value;
    }
}

// Calls on the realsense reader to process a pointcloud from the test topic
void RealsenseTestPublisher::makeProcessPointcloudCall()
{
    march_shared_msgs::GetGaitParameters service;
    service.request.realsense_category = realsense_category;
    service.request.frame_id_to_transform_to = frame_id_to_transform_to;

    // The image always comes from simulated camera topic (enum value 2)
    service.request.camera_to_use = 2;
    process_pointcloud_service_client.call(service);
}
