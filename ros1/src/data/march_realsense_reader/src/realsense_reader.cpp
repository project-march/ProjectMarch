#include <ctime>
#include <dynamic_reconfigure/server.h>
#include <map>
#include <march_realsense_reader/realsense_reader.h>
#include <march_shared_msgs/GetGaitParameters.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_processor/hull_finder.h>
#include <pointcloud_processor/parameter_determiner.h>
#include <pointcloud_processor/preprocessor.h>
#include <pointcloud_processor/region_creator.h>
#include <ros/console.h>
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;
using PlaneCoefficientsVector = std::vector<pcl::ModelCoefficients::Ptr>;
using HullVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using PolygonVector = std::vector<std::vector<pcl::Vertices>>;

std::string TOPIC_CAMERA_FRONT = "/camera_front/depth/color/points";
std::string TOPIC_CAMERA_BACK = "/camera_back/depth/color/points";
std::map<int, std::string> POINTCLOUD_TOPICS
    = { { march_shared_msgs::GetGaitParametersRequest::CAMERA_FRONT,
            TOPIC_CAMERA_FRONT },
          { march_shared_msgs::GetGaitParametersRequest::CAMERA_BACK,
              TOPIC_CAMERA_BACK } };
ros::Duration POINTCLOUD_TIMEOUT = ros::Duration(/*t=*/1.0); // secs

RealSenseReader::RealSenseReader(ros::NodeHandle* n)
    : n_(n)
    , selected_gait_(-1)
    , use_left_foot_(nullptr)
    , debugging_launch(false)
{

    // Create a subscriber for every pointcloud topic
    for (auto& item : POINTCLOUD_TOPICS) {
        pointcloud_subscribers_[item.first]
            = n_->subscribe<sensor_msgs::PointCloud2>(item.second,
                /*queue_size=*/1, &RealSenseReader::pointcloudCallback, this);
    }

    read_pointcloud_service_
        = n_->advertiseService(/*service=*/"/camera/process_pointcloud",
            &RealSenseReader::processPointcloudCallback, this);

    preprocessor_ = std::make_unique<NormalsPreprocessor>(debugging_);
    region_creator_ = std::make_unique<RegionGrower>(debugging_);
    hull_finder_ = std::make_unique<CHullFinder>(debugging_);
    parameter_determiner_
        = std::make_unique<HullParameterDeterminer>(debugging_);

    if (ros::param::get("/realsense_debug", debugging_launch)) {
        debugging_ = debugging_launch;
    } else {
        ROS_WARN("Unable to obtain debug parameter from parameter server");
    }

    if (debugging_) {
        ROS_DEBUG(
            "Realsense reader started with debugging, all intermediate result "
            "steps will be published and more information given in console, but"
            " this might slow the process, this can be turned off in the "
            "realsense_reader.launch of with dynamic reconfiguring.");
        if (ros::console::set_logger_level(
                ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>(
            "/camera/preprocessed_cloud", /*queue_size=*/1);
        region_pointcloud_publisher_
            = n_->advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                "/camera/region_cloud", /*queue_size=*/1);
        hull_marker_array_publisher_
            = n_->advertise<visualization_msgs::Marker>(
                "/camera/hull_marker_list", /*queue_size=*/1);
        hull_parameter_determiner_publisher_
            = n_->advertise<visualization_msgs::MarkerArray>(
                "/camera/foot_locations_marker_array", /*queue_size=*/1);
    }
}

void RealSenseReader::readConfigCb(
    march_realsense_reader::pointcloud_parametersConfig& config, uint32_t level)
{
    ROS_DEBUG(
        "Changed march_realsense_parameters with dynamic reconfiguration");

    // Only dynamically reconfigure debug flag if flag was true at launch
    if (debugging_launch) {
        debugging_ = config.debug;
    }

    if (not debugging_) {
        if (ros::console::set_logger_level(
                ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    preprocessor_->readParameters(config);
    region_creator_->readParameters(config);
    parameter_determiner_->readParameters(config);
    hull_finder_->readParameters(config);
}

// This method executes the logic to process a pointcloud
bool RealSenseReader::processPointcloud(const PointCloud::Ptr& pointcloud,
    march_shared_msgs::GetGaitParameters::Response& res)
{
    clock_t start_of_processing_time = clock();
    Normals::Ptr normals = boost::make_shared<Normals>();

    // Preprocess
    bool preprocessing_was_successful = preprocessor_->preprocess(
        pointcloud, normals, frame_id_to_transform_to_);
    if (debugging_) {
        ROS_DEBUG("Done preprocessing, see /camera/preprocessed_cloud for "
                  "resulting point cloud");
        publishCloud<pcl::PointXYZ>(
            preprocessed_pointcloud_publisher_, *pointcloud);
    }
    if (not preprocessing_was_successful) {
        res.error_message = "Preprocessing was unsuccessful, see debug output "
                            "for more information";
        res.success = false;
        return false;
    }

    // Setup data structures for region creating
    boost::shared_ptr<RegionVector> region_vector
        = boost::make_shared<RegionVector>();
    // Create regions
    bool region_creating_was_successful
        = region_creator_->createRegions(pointcloud, normals, region_vector);

    if (not region_creating_was_successful) {
        res.error_message
            = "Region creating was unsuccessful, see debug output "
              "for more information";
        res.success = false;
        return false;
    }

    if (debugging_) {
        ROS_DEBUG("Done creating regions, now publishing point cloud regions "
                  "to /camera/region_cloud");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured_cloud
            = region_creator_->debug_visualisation();
        publishCloud<pcl::PointXYZRGB>(
            region_pointcloud_publisher_, *coloured_cloud);
    }

    // Setup data structures for finding
    boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector
        = boost::make_shared<PlaneCoefficientsVector>();
    boost::shared_ptr<HullVector> hull_vector
        = boost::make_shared<HullVector>();
    boost::shared_ptr<PolygonVector> polygon_vector
        = boost::make_shared<PolygonVector>();
    // Find hulls
    bool hull_finding_was_successful
        = hull_finder_->findHulls(pointcloud, normals, region_vector,
            plane_coefficients_vector, hull_vector, polygon_vector);
    if (debugging_) {
        ROS_DEBUG("Done creating hulls, now publishing markers to "
                  "/camera/hull_marker_list");
        publishHullMarkerArray(hull_vector);
    }
    if (not hull_finding_was_successful) {
        res.error_message = "Hull finding was unsuccessful, see debug output "
                            "for more information";
        res.success = false;
        return false;
    }

    // Setup data structures for parameter determining
    auto selected_gait = (SelectedGait)selected_gait_;
    boost::shared_ptr<march_shared_msgs::GaitParameters> gait_parameters
        = boost::make_shared<march_shared_msgs::GaitParameters>();
    // Determine parameters
    bool parameter_determining_was_successful
        = parameter_determiner_->determineParameters(plane_coefficients_vector,
            hull_vector, polygon_vector, selected_gait, gait_parameters);
    if (debugging_) {
        ROS_DEBUG("Done determining parameters, now publishing a marker to "
                  "/camera/foot_locations_marker_array");
        publishParameterDeterminerMarkerArray();
    }
    if (not parameter_determining_was_successful) {
        res.error_message
            = "Parameter determining was unsuccessful, see debug output "
              "for more information";
        res.success = false;
        return false;
    }

    res.gait_parameters = *gait_parameters;

    clock_t end_of_processing_time = clock();

    double time_taken
        = double(end_of_processing_time - start_of_processing_time)
        / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by point cloud processor is : "
        << std::fixed << time_taken << std::setprecision(5) << " sec "
        << std::endl);

    res.success = true;
    // Returning false means that the service was not able to respond at all,
    // this causes problems with the bridge, therefore always return true!
    return true;
}

// Publish a pointcloud of any point type on a publisher
template <typename T>
void RealSenseReader::publishCloud(
    const ros::Publisher& publisher, pcl::PointCloud<T> cloud)
{
    cloud.width = 1;
    cloud.height = cloud.points.size();

    sensor_msgs::PointCloud2 msg;

    pcl::toROSMsg(cloud, msg);

    msg.header.frame_id = frame_id_to_transform_to_;

    publisher.publish(msg);
}

// Turn a HullVector into a marker with a list of points and publish for
// visualization
void RealSenseReader::publishHullMarkerArray(
    const boost::shared_ptr<HullVector>& hull_vector)
{
    visualization_msgs::Marker marker_list;
    marker_list.header.frame_id = frame_id_to_transform_to_;
    marker_list.header.stamp = ros::Time::now();
    marker_list.ns = "hulls";
    marker_list.action = visualization_msgs::Marker::ADD;
    marker_list.pose.orientation.w = 1.0;
    marker_list.id = 0;
    marker_list.type = visualization_msgs::Marker::CUBE_LIST;
    float cube_side_length = 0.07;
    marker_list.scale.x = cube_side_length;
    marker_list.scale.y = cube_side_length;
    marker_list.scale.z = cube_side_length;

    for (const pcl::PointCloud<pcl::PointXYZ>::Ptr& hull : *hull_vector) {
        // Color the hull with a random color (r, g and b in [1, 0])
        int number_of_colors = 500;

        // clang-tidy linter cert-msc30-c and cert-msc50-cpp say that rand() is
        // not a uniform distribution. This is not something that is important
        // here, therefore these lines can ignore this linter rule.
        // NOLINTNEXTLINE(cert-msc30-c, cert-msc50-cpp)
        double r = (rand() % number_of_colors) / (double)number_of_colors;
        // NOLINTNEXTLINE(cert-msc30-c, cert-msc50-cpp)
        double g = (rand() % number_of_colors) / (double)number_of_colors;
        // NOLINTNEXTLINE(cert-msc30-c, cert-msc50-cpp)
        double b = (rand() % number_of_colors) / (double)number_of_colors;
        for (pcl::PointXYZ hull_point : *hull) {
            geometry_msgs::Point marker_point;
            marker_point.x = hull_point.x;
            marker_point.y = hull_point.y;
            marker_point.z = hull_point.z;

            std_msgs::ColorRGBA marker_color;
            marker_color.r = r;
            marker_color.g = g;
            marker_color.b = b;
            marker_color.a = 1.0;

            marker_list.points.push_back(marker_point);
            marker_list.colors.push_back(marker_color);
        }
    }
    hull_marker_array_publisher_.publish(marker_list);
}

// Create markers from the parameter determiner and publish them for
// visualization
void RealSenseReader::publishParameterDeterminerMarkerArray()
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker optimal_foot_location_marker;
    optimal_foot_location_marker.id = 0;
    optimal_foot_location_marker.header.frame_id = frame_id_to_transform_to_;
    fillOptimalFootLocationMarker(parameter_determiner_->optimal_foot_location,
        optimal_foot_location_marker);

    visualization_msgs::Marker foot_locations_to_try_marker_list;
    foot_locations_to_try_marker_list.id = 1;
    foot_locations_to_try_marker_list.header.frame_id
        = frame_id_to_transform_to_;
    fillFootLocationsToTryMarker(parameter_determiner_->foot_locations_to_try,
        foot_locations_to_try_marker_list);

    visualization_msgs::Marker possible_foot_locations_marker_list;
    possible_foot_locations_marker_list.id = 2;
    possible_foot_locations_marker_list.header.frame_id
        = frame_id_to_transform_to_;
    fillPossibleFootLocationsMarker(
        parameter_determiner_->possible_foot_locations,
        parameter_determiner_->optimal_foot_location,
        possible_foot_locations_marker_list);

    marker_array.markers.push_back(optimal_foot_location_marker);
    marker_array.markers.push_back(foot_locations_to_try_marker_list);
    marker_array.markers.push_back(possible_foot_locations_marker_list);

    hull_parameter_determiner_publisher_.publish(marker_array);
}

void RealSenseReader::fillPossibleFootLocationsMarker(
    PointNormalCloud::Ptr const& possible_foot_locations,
    pcl::PointNormal const optimal_foot_location,
    visualization_msgs::Marker& marker_list)
{
    marker_list.pose.orientation.w = 1.0;
    marker_list.type = visualization_msgs::Marker::SPHERE_LIST;
    float sphere_radius = 0.05;
    marker_list.scale.x = sphere_radius;
    marker_list.scale.y = sphere_radius;
    marker_list.scale.z = sphere_radius;

    for (pcl::PointNormal ground_point : *possible_foot_locations) {
        geometry_msgs::Point marker_point;
        marker_point.x = ground_point.x;
        marker_point.y = ground_point.y;
        marker_point.z = ground_point.z;

        std_msgs::ColorRGBA marker_color;
        marker_color.r = 0.0;
        marker_color.g = 1.0;
        marker_color.b = 0.0;
        marker_color.a = 0.7;

        marker_list.points.push_back(marker_point);
        marker_list.colors.push_back(marker_color);
    }
}

// Create a marker list from the 'foot locations to try' and publish it and
// publish for visualization
void RealSenseReader::fillFootLocationsToTryMarker(
    PointCloud2D::Ptr const& foot_locations_to_try,
    visualization_msgs::Marker& marker_list)
{
    marker_list.pose.orientation.w = 1.0;
    marker_list.type = visualization_msgs::Marker::SPHERE_LIST;
    float sphere_radius = 0.05;
    marker_list.scale.x = sphere_radius;
    marker_list.scale.y = sphere_radius;
    marker_list.scale.z = sphere_radius;

    for (pcl::PointXY ground_point : *foot_locations_to_try) {

        geometry_msgs::Point marker_point;
        marker_point.x = ground_point.x;
        marker_point.y = ground_point.y;
        marker_point.z = 0;

        std_msgs::ColorRGBA marker_color;
        marker_color.r = 0.0;
        marker_color.g = 0.0;
        marker_color.b = 1.0;
        marker_color.a = 0.7;

        marker_list.points.push_back(marker_point);
        marker_list.colors.push_back(marker_color);
    }
}

// Create a marker from the optimal foot location and publish it and publish for
// visualization
void RealSenseReader::fillOptimalFootLocationMarker(
    pcl::PointNormal const optimal_foot_location,
    visualization_msgs::Marker& marker)
{
    marker.type = visualization_msgs::Marker::SPHERE;
    float sphere_radius = 0.07;
    marker.scale.x = sphere_radius;
    marker.scale.y = sphere_radius;
    marker.scale.z = sphere_radius;
    marker.pose.position.x = optimal_foot_location.x;
    marker.pose.position.y = optimal_foot_location.y;
    marker.pose.position.z = optimal_foot_location.z;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
}

// The callback for the service that was starts processing the point cloud and
// gives back parameters for a gait
bool RealSenseReader::processPointcloudCallback(
    march_shared_msgs::GetGaitParameters::Request& req,
    march_shared_msgs::GetGaitParameters::Response& res)
{
    selected_gait_ = req.selected_gait;
    frame_id_to_transform_to_ = req.frame_id_to_transform_to;

    time_t start_callback = clock();
    if (req.camera_to_use >= POINTCLOUD_TOPICS.size()) {
        res.error_message
            = "Unknown camera given in the request, not available in the "
              "POINTCLOUD_TOPICS in the realsense_reader";
        res.success = false;
        return true;
    }
    boost::shared_ptr<const sensor_msgs::PointCloud2> input_cloud
        = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            POINTCLOUD_TOPICS[req.camera_to_use], *n_, POINTCLOUD_TIMEOUT);

    if (input_cloud == nullptr) {
        res.error_message = "No pointcloud published within timeout, so "
                            "no processing could be done.";
        res.success = false;
        return false;
    }

    PointCloud converted_cloud;
    pcl::fromROSMsg(*input_cloud, converted_cloud);
    PointCloud::Ptr point_cloud
        = boost::make_shared<PointCloud>(converted_cloud);

    bool success = processPointcloud(point_cloud, res);

    time_t end_callback = clock();
    double time_taken
        = double(end_callback - start_callback) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by process point cloud callback method: "
        << std::fixed << time_taken << std::setprecision(5) << " sec "
        << std::endl);

    return success;
}
