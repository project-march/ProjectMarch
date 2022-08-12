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
std::string CAMERA_FRAME_ID_BACK = "camera_back_depth_optical_frame";
std::string PROCESS_POINTCLOUD_SERVICE_NAME = "/camera/process_pointcloud";
std::string PUBLISH_POINTCLOUD_SERVICE_NAME = "/camera/publish_test_cloud";
std::string DATASET_CONFIGURATION_NAME = "pointcloud_information.yaml";
std::string POINTCLOUD_EXTENSION = ".ply";
ros::Duration POINTCLOUD_TIMEOUT = ros::Duration(/*t=*/1.0); // secs

RealsenseTestPublisher::RealsenseTestPublisher(ros::NodeHandle* n)
    : n_(n)
    , should_publish(false)
    , from_back_camera(false)
    , selected_mode((SelectedMode)-1)
    , from_realsense_viewer(nullptr)
    , save_camera_back(false)
    , realsense_category(-1)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    path directory_path = ros::package::getPath("march_realsense_test_publisher");
    path relative_data_path(/*__source*/ "config/datasets/");

    // The source directory should be two steps behind this source file
    path source_file_path(__FILE__);
    write_path = source_file_path.parent_path().parent_path() / relative_data_path;

    updateFileNamesVector();

    if (file_names.size() == 0) {
        ROS_ERROR_STREAM(
            "There are no .ply files present under path " << write_path << ". Shutting down the test publisher.");
        ros::shutdown();
    }
    publish_test_cloud_service = n_->advertiseService(
        /*service=*/"/camera/publish_test_cloud", &RealsenseTestPublisher::publishTestDatasetCallback, this);

    test_cloud_publisher = n_->advertise<PointCloud>(TOPIC_TEST_CLOUDS, /*queue_size=*/1);

    process_pointcloud_service_client
        = n_->serviceClient<march_shared_msgs::GetGaitParameters>(PROCESS_POINTCLOUD_SERVICE_NAME);

    config_tree = loadConfig(DATASET_CONFIGURATION_NAME);
}

void RealsenseTestPublisher::updateFileNamesVector()
{
    for (const auto& entry : std::filesystem::directory_iterator(write_path)) {
        if (std::filesystem::is_regular_file(entry) && entry.path().extension() == POINTCLOUD_EXTENSION) {
            file_names.push_back(entry.path().filename().string());
        }
    }
}

YAML::Node RealsenseTestPublisher::loadConfig(const std::string& config_file)
{
    YAML::Node config_tree;
    std::string path = ros::package::getPath("march_realsense_test_publisher") + "/config/" + config_file;
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
    march_shared_msgs::PublishTestDataset::Request& req, march_shared_msgs::PublishTestDataset::Response& res)
{
    selected_mode = (SelectedMode)req.selected_mode;
    // Only update the pointcloud file name from the service if it is relevant
    if (selected_mode == SelectedMode::custom) {
        pointcloud_file_name = req.pointcloud_file_name;
    }
    if (selected_mode == SelectedMode::save) {
        save_pointcloud_name = req.pointcloud_file_name;
        save_camera_back = req.save_camera_back;
    }
    updatePublishLoop(res);
    return true;
}

void RealsenseTestPublisher::getProcessPointcloudInputs()
{
    if (YAML::Node pointcloud_config = config_tree[pointcloud_file_name]) {
        realsense_category = yaml_utilities::grabParameter<int>(pointcloud_config, "realsense_category").value();
        from_back_camera = yaml_utilities::grabParameter<bool>(pointcloud_config, "from_back_camera").value();
        subgait_name = yaml_utilities::grabParameter<std::string>(pointcloud_config, "subgait_name").value();
        from_realsense_viewer = yaml_utilities::grabParameter<bool>(pointcloud_config, "from_realsense_viewer").value();
    } else {
        ROS_WARN_STREAM("No configuration specified for pointcloud file with name "
            << pointcloud_file_name << ". Continuing with default parameters");
        realsense_category = 0;
        from_back_camera = false;
        subgait_name = "right_open";
        from_realsense_viewer = false;
    }
}

// Sets the right cloud as the pointcloud to publish based on the file name
bool RealsenseTestPublisher::loadPointcloudToPublishFromFilename()
{
    getProcessPointcloudInputs();

    pointcloud_to_publish = boost::make_shared<PointCloud>();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(write_path.string() + pointcloud_file_name, *pointcloud_to_publish) == -1) {
        ROS_WARN_STREAM("Couldn't find file at " << write_path.string() + pointcloud_file_name
                                                 << ". file name must be one of " << getFileNamesString());
        return false;
    }

    if (from_back_camera) {
        pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID_BACK;
    } else {
        pointcloud_to_publish->header.frame_id = CAMERA_FRAME_ID_FRONT;
    }
    // There is a weird inconsistency between the coordinate systems of the the
    // realsense viewer and that of rviz, this makes them consistent
    if (from_realsense_viewer) {
        transformToCameraCoordinates();
    }
    return true;
}

// Publish the current pointcloud to publish
void RealsenseTestPublisher::publishTestCloud(const ros::TimerEvent& timer_event)
{
    if (should_publish) {
        pcl_conversions::toPCL(ros::Time::now(), pointcloud_to_publish->header.stamp);
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
bool RealsenseTestPublisher::startPublishingPointclouds()
{
    // Start at a random index to reduce over fitting on one data set
    int random_index = rand() % file_names.size(); // NOLINT
    pointcloud_file_name = file_names[random_index];
    return loadPointcloudToPublishFromFilename();
}

// Publishes the next pointcloud in the dataset directory
bool RealsenseTestPublisher::publishNextPointcloud()
{
    // If already publishing, find the next pointcloud and publish that
    // Otherwise, start publishing
    if (should_publish) {
        // find the current pointcloud filename
        auto filename_iterator = std::find(file_names.begin(), file_names.end(), pointcloud_file_name);

        // Set the current pointcloud file name to the next name in the list, if
        // the old name is the last in the list, set the new name to the first
        // in the list
        if (filename_iterator == file_names.end()) {
            ROS_WARN_STREAM("The old pointcloud file name could not be found in "
                            "the file name vector. Unable to find next pointcloud.");
            return false;
        } else if (filename_iterator == file_names.end() - 1) {
            pointcloud_file_name = file_names[0];
        } else {
            pointcloud_file_name = *(filename_iterator + 1);
        }
        return loadPointcloudToPublishFromFilename();
    } else {
        return startPublishingPointclouds();
    }
}

bool RealsenseTestPublisher::saveCurrentPointcloud()
{
    // Set the correct pointcloud topic
    std::string pointcloud_topic;
    if (save_camera_back) {
        pointcloud_topic = TOPIC_CAMERA_BACK;
    } else {
        pointcloud_topic = TOPIC_CAMERA_FRONT;
    }
    if (save_pointcloud_name.compare(save_pointcloud_name.length() - POINTCLOUD_EXTENSION.length(),
            POINTCLOUD_EXTENSION.length(), POINTCLOUD_EXTENSION)
        != 0) {
        ROS_WARN_STREAM("The name under which to save the pointcloud should "
                        "end with .ply but does not. Supplied name is "
            << save_pointcloud_name);
        return false;
    }
    // get the next pointcloud from the topic
    boost::shared_ptr<const sensor_msgs::PointCloud2> input_cloud
        = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pointcloud_topic, *n_, POINTCLOUD_TIMEOUT);

    if (input_cloud == nullptr) {
        ROS_WARN_STREAM("No pointcloud published within timeout on topic " << pointcloud_topic
                                                                           << ", so "
                                                                              "no saving could be done.");
        return false;
    }
    PointCloud converted_cloud;
    pcl::fromROSMsg(*input_cloud, converted_cloud);
    PointCloud::Ptr point_cloud = boost::make_shared<PointCloud>(converted_cloud);

    if (pcl::io::savePLYFileBinary(write_path.string() + save_pointcloud_name, *point_cloud) == -1) {
        return false;
    }
    updateFileNamesVector();
    return true;
}

// Publish the right pointcloud based on the latest service call
void RealsenseTestPublisher::updatePublishLoop(march_shared_msgs::PublishTestDataset::Response& res)
{
    // The update is successful by default until something goes wrong
    bool success = true;
    std::string info_message;
    std::string warn_message;

    switch (selected_mode) {
        case SelectedMode::next: {
            if (should_publish = publishNextPointcloud()) {
                info_message = "Now publishing a pointcloud with file name " + pointcloud_file_name
                    + " and processing the cloud.";
                makeProcessPointcloudCall();
            } else {
                warn_message = "failed to publish a pointcloud with file name " + pointcloud_file_name;
            }
            success = should_publish;
            break;
        }
        case SelectedMode::custom: {
            if (should_publish = loadPointcloudToPublishFromFilename()) {
                info_message = "Now publishing a pointcloud with file name " + pointcloud_file_name
                    + " and processing the cloud.";
                makeProcessPointcloudCall();
            } else {
                warn_message = "failed to publish a pointcloud with file name " + pointcloud_file_name;
            }
            success = should_publish;
            break;
        }
        case SelectedMode::end: {
            should_publish = false;
            info_message = "Stopped publishing pointclouds";
            success = true;
            break;
        }
        case SelectedMode::save: {
            if (success = saveCurrentPointcloud()) {
                info_message = "Succesfully saved pointcloud as " + write_path.string() + save_pointcloud_name;
            } else {
                warn_message = "Failed to save pointcloud";
            }
            break;
        }
        default: {
            warn_message = "Invalid mode selected";
            success = false;
            return;
        }
    }
    if (success) {
        res.message = info_message;
        ROS_DEBUG_STREAM(info_message);
    } else {
        res.message = warn_message + ". See the ros1 terminal for more information.";
        ROS_WARN_STREAM(warn_message);
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
    service.request.subgait_name = subgait_name;

    // The image always comes from simulated camera topic (enum value 2)
    service.request.camera_to_use = 2;
    process_pointcloud_service_client.call(service);
}
