#include <cmath>
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
using PointsVector = std::vector<PointCloud::Ptr>;
using NormalsVector = std::vector<Normals::Ptr>;
using PlaneCoefficientsVector = std::vector<pcl::ModelCoefficients::Ptr>;
using HullVector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>;
using PolygonVector = std::vector<std::vector<pcl::Vertices>>;
using PointCloud2D = pcl::PointCloud<pcl::PointXY>;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;

std::string TOPIC_CAMERA_FRONT = "/camera_front/depth/color/points";
std::string TOPIC_CAMERA_BACK = "/camera_back/depth/color/points";
std::string TOPIC_TEST_CLOUDS = "/test_clouds";

std::map<int, std::string> POINTCLOUD_TOPICS
    = { { march_shared_msgs::GetGaitParametersRequest::CAMERA_FRONT, TOPIC_CAMERA_FRONT },
          { march_shared_msgs::GetGaitParametersRequest::CAMERA_BACK, TOPIC_CAMERA_BACK },
          { march_shared_msgs::GetGaitParametersRequest::TEST_CLOUD, TOPIC_TEST_CLOUDS } };
ros::Duration POINTCLOUD_TIMEOUT = ros::Duration(/*t=*/1.0); // secs

std::map<std::string, std::string> SUBGAIT_NAME_TO_REALSENSE_FRAME_ID_MAP
    = { { /*__x=*/"right_open", /*__y=*/"foot_right" }, { /*__x=*/"left_open", /*__y=*/"foot_left" },
          { /*__x=*/"right_swing", /*__y=*/"foot_right" }, { /*__x=*/"left_swing", /*__y=*/"foot_left" },
          { /*__x=*/"right_close", /*__y=*/"foot_right" }, { /*__x=*/"left_close", /*__y=*/"foot_left" },
          { /*__x=*/"prepare_sit_down", /*__y=*/"foot_left" } };

RealSenseReader::RealSenseReader(ros::NodeHandle* n)
    : n_(n)
    , realsense_category_(-1)
    , use_left_foot_(nullptr)
    , debugging_launch(false)
    , publish_hull_area_debug_(nullptr)
{

    // Create a subscriber for every pointcloud topic
    for (auto& item : POINTCLOUD_TOPICS) {
        pointcloud_subscribers_[item.first] = n_->subscribe<sensor_msgs::PointCloud2>(item.second,
            /*queue_size=*/1, &RealSenseReader::pointcloudCallback, this);
    }

    read_pointcloud_service_ = n_->advertiseService(
        /*service=*/"/camera/process_pointcloud", &RealSenseReader::processPointcloudCallback, this);

    preprocessor_ = std::make_unique<NormalsPreprocessor>(debugging_);
    region_creator_ = std::make_unique<RegionGrower>(debugging_);
    hull_finder_ = std::make_unique<CHullFinder>(debugging_);
    parameter_determiner_ = std::make_unique<HullParameterDeterminer>(debugging_);

    if (ros::param::get("/realsense_debug", debugging_launch)) {
        debugging_ = debugging_launch;
    } else {
        ROS_WARN("Unable to obtain debug parameter from parameter server");
    }

    if (debugging_) {
        ROS_DEBUG("Realsense reader started with debugging, all intermediate result "
                  "steps will be published and more information given in console, but"
                  " this might slow the process, this can be turned off in the "
                  "realsense_reader.launch of with dynamic reconfiguring.");
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>("/camera/preprocessed_cloud", /*queue_size=*/1);
        region_pointcloud_publisher_
            = n_->advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/region_cloud", /*queue_size=*/1);
        hull_marker_array_publisher_
            = n_->advertise<visualization_msgs::Marker>("/camera/hull_marker_list", /*queue_size=*/1);
        hull_area_pointcloud_publisher_ = n_->advertise<PointNormalCloud>("/camera/hull_area_cloud", /*queue_size=*/1);
        hull_parameter_determiner_publisher_
            = n_->advertise<visualization_msgs::MarkerArray>("/camera/foot_locations_marker_array", /*queue_size=*/1);
    }
}

void RealSenseReader::readConfigCb(march_realsense_reader::pointcloud_parametersConfig& config, uint32_t level)
{
    ROS_DEBUG("Changed march_realsense_parameters with dynamic reconfiguration");

    // Only dynamically reconfigure debug flag if flag was true at launch
    if (debugging_launch) {
        debugging_ = config.debug;
    }

    if (not debugging_) {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    publish_hull_area_debug_ = config.publish_hull_area_debug;

    preprocessor_->readParameters(config);
    region_creator_->readParameters(config);
    parameter_determiner_->readParameters(config);
    hull_finder_->readParameters(config);
}

// This method executes the logic to process a pointcloud
void RealSenseReader::processPointcloud(
    const PointCloud::Ptr& pointcloud, march_shared_msgs::GetGaitParameters::Response& res)
{
    Normals::Ptr normals = boost::make_shared<Normals>();
    auto realsense_category = (RealSenseCategory)realsense_category_;
    // Preprocess
    bool preprocessing_was_successful
        = preprocessor_->preprocess(pointcloud, normals, realsense_category, frame_id_to_transform_to_);

    if (not preprocessing_was_successful) {
        res.error_message = "Preprocessing was unsuccessful, see debug output "
                            "for more information";
        res.success = false;
        return;
    }

    if (debugging_) {
        ROS_DEBUG("Done preprocessing, see /camera/preprocessed_cloud for "
                  "resulting point cloud");
        publishCloud<pcl::PointXYZ>(preprocessed_pointcloud_publisher_, *pointcloud);
    }

    // Setup data structures for region creating
    boost::shared_ptr<PointsVector> points_vector = boost::make_shared<PointsVector>();
    boost::shared_ptr<NormalsVector> normals_vector = boost::make_shared<NormalsVector>();
    // Create regions
    bool region_creating_was_successful
        = region_creator_->createRegions(pointcloud, normals, realsense_category, points_vector, normals_vector);

    if (not region_creating_was_successful) {
        res.error_message = "Region creating was unsuccessful, see debug output "
                            "for more information";
        res.success = false;
        return;
    }
    if (debugging_) {
        ROS_DEBUG("Done creating regions, now publishing point cloud regions "
                  "to /camera/region_cloud");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured_cloud = region_creator_->debug_visualisation();
        publishCloud<pcl::PointXYZRGB>(region_pointcloud_publisher_, *coloured_cloud);
    }

    // Setup data structures for finding hulls
    boost::shared_ptr<PlaneCoefficientsVector> plane_coefficients_vector
        = boost::make_shared<PlaneCoefficientsVector>();
    boost::shared_ptr<HullVector> hull_vector = boost::make_shared<HullVector>();
    boost::shared_ptr<PolygonVector> polygon_vector = boost::make_shared<PolygonVector>();
    // Find hulls
    bool hull_finding_was_successful = hull_finder_->findHulls(
        pointcloud, normals, points_vector, normals_vector, plane_coefficients_vector, hull_vector, polygon_vector);

    if (not hull_finding_was_successful) {
        res.error_message = "Hull finding was unsuccessful, see debug output "
                            "for more information";
        res.success = false;
        return;
    }

    if (debugging_) {
        ROS_DEBUG("Done creating hulls, now publishing markers to "
                  "/camera/hull_marker_list");
        publishHullMarkerArray(hull_vector);
    }

    // Setup data structures for parameter determining
    boost::shared_ptr<march_shared_msgs::GaitParameters> gait_parameters
        = boost::make_shared<march_shared_msgs::GaitParameters>();
    // Determine parameters
    bool parameter_determining_was_successful = parameter_determiner_->determineParameters(plane_coefficients_vector,
        hull_vector, polygon_vector, realsense_category, gait_parameters, frame_id_to_transform_to_, subgait_name_);
    // The debug output of the parameter determiner is made so that it can be
    // visualized even if the parameter determiner was not successful
    if (debugging_) {
        ROS_DEBUG("Done determining parameters, now publishing a marker array to "
                  "/camera/foot_locations_marker_array. The color coding of the "
                  "marker array is: \n"
                  "Blue: a foot location to try \n"
                  "Yellow: a potential foot location which is outside the reachable "
                  "area \n"
                  "Purple: a potential foot locations which is invalid for a gait "
                  "specific reason "
                  "(i.e. lacking foot support or too far removed from the reachable "
                  "positions \n"
                  "Red: Gait information such as end points of gaits \n"
                  "Green: A valid foot location \n"
                  "White: The optimal foot location \n"
                  "Ramp gait is colored based on the orientation of the surface");
        hull_parameter_determiner_publisher_.publish(parameter_determiner_->debug_marker_array);
        if (publish_hull_area_debug_) {
            ROS_DEBUG("Publishing hull area information to "
                      "/camera/hull_area_cloud");
            publishHullAreaCloud();
        }
    }
    if (not parameter_determining_was_successful) {
        res.error_message = "Parameter determining was unsuccessful, see debug output "
                            "for more information";
        res.success = false;
        return;
    }

    res.gait_parameters = *gait_parameters;

    res.success = true;
    // Returning false means that the service was not able to respond at all,
    // this causes problems with the bridge, therefore always return true!
    return;
}

// Publish a pointcloud of any point type on a publisher
template <typename T> void RealSenseReader::publishCloud(const ros::Publisher& publisher, pcl::PointCloud<T> cloud)
{
    cloud.width = 1;
    cloud.height = cloud.points.size();

    sensor_msgs::PointCloud2 msg;

    pcl::toROSMsg(cloud, msg);

    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    publisher.publish(msg);
}

// Give an idea of the area of the hulls by cropping a large input grid to the
// hull vector
void RealSenseReader::publishHullAreaCloud()
{
    // Create the input cloud to be a grid on the ground
    PointCloud::Ptr ground_cloud = boost::make_shared<PointCloud>();
    // The grid size
    float x_grid_size = 0.05;
    float y_grid_size = 0.05;
    // The area lengths the grid should span
    float min_x = -3;
    float max_x = 0;
    float min_y = -1.5;
    float max_y = 1.5;
    // The number of points in each direction of the grid
    int x_points = int(round((max_x - min_x) / x_grid_size)) + 1;
    int y_points = int(round((max_y - min_y) / y_grid_size)) + 1;
    for (int i = 0; i < x_points; ++i) {
        for (int j = 0; j < y_points; ++j) {
            pcl::PointXYZ point {};
            point.x = min_x + (float)i * x_grid_size;
            point.y = min_y + (float)j * y_grid_size;
            ground_cloud->push_back(point);
        }
    }
    // Crop the input (ground) cloud to the hull vector and publish the results
    PointNormalCloud::Ptr cropped_cloud = boost::make_shared<PointNormalCloud>();
    parameter_determiner_->cropCloudToHullVector(ground_cloud, cropped_cloud);
    publishCloud(hull_area_pointcloud_publisher_, *cropped_cloud);
}

// Turn a HullVector into a marker with a list of points and publish for
// visualization
void RealSenseReader::publishHullMarkerArray(const boost::shared_ptr<HullVector>& hull_vector)
{
    visualization_msgs::Marker marker_list;
    marker_list.header.frame_id = "world";
    marker_list.header.stamp = ros::Time::now();
    marker_list.ns = "hulls";
    marker_list.action = visualization_msgs::Marker::ADD;
    marker_list.pose.orientation.w = 1.0;
    marker_list.id = 0;
    marker_list.type = visualization_msgs::Marker::CUBE_LIST;
    float cube_side_length = 0.02;
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

// The callback for the service that was starts processing the point cloud and
// gives back parameters for a gait. This function should always return true,
// otherwise this gives problems with the bridge. The response can be altered
// to show if the processing was successful.
bool RealSenseReader::processPointcloudCallback(
    march_shared_msgs::GetGaitParameters::Request& req, march_shared_msgs::GetGaitParameters::Response& res)
{
    realsense_category_ = req.realsense_category;
    subgait_name_ = req.subgait_name;
    try {
        frame_id_to_transform_to_ = SUBGAIT_NAME_TO_REALSENSE_FRAME_ID_MAP.at(subgait_name_);
    } catch (std::out_of_range& ex) {
        res.error_message = "Provided subgait name " + subgait_name_
            + " has no known associated frame id "
              "to transform to.";
        ROS_WARN_STREAM(res.error_message);
        res.success = false;
        return true;
    }

    time_t start_callback = clock();
    if (req.camera_to_use >= POINTCLOUD_TOPICS.size()) {
        res.error_message = "Unknown camera given in the request, not available in the "
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
        ROS_WARN_STREAM(res.error_message);
        res.success = false;
        return true;
    }
    PointCloud converted_cloud;
    pcl::fromROSMsg(*input_cloud, converted_cloud);
    PointCloud::Ptr point_cloud = boost::make_shared<PointCloud>(converted_cloud);

    clock_t start_of_processing_time = clock();

    processPointcloud(point_cloud, res);

    clock_t end_of_processing_time = clock();

    double time_taken_by_processing
        = double(end_of_processing_time - start_of_processing_time) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by point cloud processor is : " << std::fixed << time_taken_by_processing
                                                                 << std::setprecision(5) << " sec " << std::endl);

    time_t end_callback = clock();
    double time_taken_by_callback = double(end_callback - start_callback) / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by process point cloud callback method: "
        << std::fixed << time_taken_by_callback << std::setprecision(5) << " sec " << std::endl);
    // Always return true, to show that the service was able to handle the
    // request. Otherwise, the bridge will fail.
    return true;
}
