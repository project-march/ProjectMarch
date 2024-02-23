#include "processing/cameras_interface.h"

//#include "utilities/publish_utilities.hpp"

// TODO: Change to processing_utils.h ?
//#include "utilities/realsense_to_pcl.hpp"
#include <iostream>
#include <string>
//#include <tf2_eigen/tf2_eigen.h>

// TODO: Fix this visualization msg
//#include <visualization_msgs/msg/marker_array.hpp>

// TODO: Don't think I need this 
//using namespace marchPublishUtilities;

// No lint is used to allow uninitialized variables (ros parameters)
CamerasInterface::CamerasInterface(rclcpp::Node* n,
    const std::string& left_or_right) // NOLINT
    : n_(n)
    , left_or_right_(left_or_right)
{

    // TODO: Change serial numbers, also what is a "switch_factor"?; Maybe get the serials from YAML 
    if (left_or_right_ == "left") {
        other_side_ = "right";
        switch_factor_ = -1;
        serial_number_ = "944622074337";
    } else {
        other_side_ = "left";
        switch_factor_ = 1;
        serial_number_ = "944622071535";
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(n_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // preprocessor_ = std::make_unique<Preprocessor>(n_, left_or_right_, tf_listener_, tf_buffer_);
    // point_finder_ = std::make_unique<PointFinder>(n_, left_or_right_);

    // current_frame_id_ = "toes_" + left_or_right_ + "_aligned";
    // other_frame_id_ = "toes_" + other_side_ + "_aligned";
    ORIGIN = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    last_height_ = 0;
    refresh_last_height_ = 0;
    last_frame_time_ = std::clock();
    frame_wait_counter_ = 0;
    frame_timeout_ = 5.0;
    locked_ = false;

    realsense_callback_group_ = n_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    point_callback_group_ = n_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions realsense_callback_options_;
    realsense_callback_options_.callback_group = realsense_callback_group_;
    rclcpp::SubscriptionOptions point_callback_options_;
    point_callback_options_.callback_group = point_callback_group_;

    // TODO: Change the topic name (+ maybe the color to just depth points)
    topic_camera_front_ = "/camera_front_" + left_or_right + "/depth/color/points";
    topic_other_chosen_point_ = "/march/chosen_foot_position/" + other_side_; // in current_frame_id
    topic_current_chosen_point_ = "/march/chosen_foot_position/" + left_or_right_;

    point_publisher_ = n_->create_publisher<march_shared_msgs::msg::FootPosition>(
        "/march/foot_position/" + left_or_right_, /*qos=*/1);

    // TODO: This is supposed to be 

    // preprocessed_pointcloud_publisher_
    //     = n_->create_publisher<sensor_msgs::msg::PointCloud2>("/camera_" + left_or_right_ + "/preprocessed_cloud",
    //         /*qos=*/1);
    point_marker_publisher_ = n_->create_publisher<visualization_msgs::msg::Marker>(
        "/camera_" + left_or_right_ + "/found_points", /*qos=*/1);


    // TODO: See if we are going to use these for footstep planning
    foot_gap_ = n_->get_parameter("foot_gap").as_double();
    step_distance_ = n_->get_parameter("step_distance").as_double();
    sample_size_ = n_->get_parameter("sample_size").as_int();

    outlier_distance_ = n_->get_parameter("outlier_distance").as_double();
    height_zero_threshold_ = n_->get_parameter("height_zero_threshold").as_double();
    height_distance_coefficient_ = n_->get_parameter("height_distance_coefficient").as_double();
    realsense_simulation_ = n_->get_parameter("realsense_simulation").as_bool();
    found_points_.resize(sample_size_);
    displacements_ = point_finder_->getDisplacements();

    // Process the actual cameras' data
    if (!realsense_simulation_) {
        while (true) {
            try {
                config_.enable_device(serial_number_);
                config_.enable_stream(RS2_STREAM_DEPTH, /*width=*/640,
                    /*height=*/480, RS2_FORMAT_Z16, /*framerate=*/15);
                pipe_.start(config_);
            } catch (const rs2::error& e) {
                std::string error_message = e.what();
                RCLCPP_WARN(n_->get_logger(), "Error while initializing %s RealSense camera: %s",
                    left_or_right_.c_str(), error_message.c_str());
                rclcpp::sleep_for(/*nanoseconds=*/std::chrono::nanoseconds(1000000000)); // 1 second
                continue;
            }

            realsense_timer_ = n_->create_wall_timer(
                std::chrono::milliseconds(30),
                [this]() -> void {
                    processRealSenseDepthFrames();
                },
                realsense_callback_group_);

            RCLCPP_INFO(n_->get_logger(), "\033[1;36m%s RealSense connected (%s) \033[0m", left_or_right_.c_str(),
                serial_number_.c_str());

            break;
        }
    } else {
        // Initialize the callback for the RealSense simulation plugin
        std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> callback
            = std::bind(&FootPositionFinder::processSimulatedDepthFrames, this, std::placeholders::_1);

        // TODO: Change the topic here
        pointcloud_subscriber_ = n_->create_subscription<sensor_msgs::msg::PointCloud2>(topic_camera_front_,
            /*qos=*/1, callback, realsense_callback_options_);

        RCLCPP_INFO(
            n_->get_logger(), "\033[1;36mSimulated RealSense callback initialized (%s)\033[0m", left_or_right_.c_str());
    }
}

/**
 * Retrieve parameter values that are dynamically reconfigured and update the
 * values in the class.
 *
 * @param parameters Instance containing updated parameters.processSimulatedDepthFrames
 */
void CamerasInterface::readParameters(const std::vector<rclcpp::Parameter>& parameters) {

    for (const auto& param : parameters) {
        const std::string& name = param.get_name();
        
        switch(hash(name.c_str())) {
            case hash("foot_gap"):
                foot_gap_ = param.as_double();
                break;
            case hash("step_distance"):
                step_distance_ = param.as_double();
                break;
            case hash("sample_size"):
                sample_size_ = param.as_int();
                break;
            case hash("outlier_distance"):
                outlier_distance_ = param.as_double();
                break;
            case hash("height_zero_threshold"):
                height_zero_threshold_ = param.as_double();
                break;
            case hash("realsense_simulation"):
                realsense_simulation_ = param.as_bool();
                break;
            case hash("height_distance_coefficient"):
                height_distance_coefficient_ = param.as_double();
                break;
            default:
                // TODO: Handle unknown parameters
                break;
        }

        RCLCPP_INFO(n_->get_logger(), "\033[92mParameter %s updated in %s Foot Position Finder\033[0m",
            param.get_name().c_str(), left_or_right_.c_str());
    }


    found_points_.resize(sample_size_);
    point_finder_->readParameters(parameters);
    // displacements_ = point_finder_->getDisplacements();
}

/**
 * Listen for RealSense frames from a camera, apply filters to them and process
 * the eventual pointcloud.
 */
void CamerasInterface::processRealSenseDepthFrames() {

    float difference = float(std::clock() - last_frame_time_) / CLOCKS_PER_SEC;

    if ((int)(difference / frame_timeout_) > frame_wait_counter_) {
    
        frame_wait_counter_++;
        RCLCPP_WARN(n_->get_logger(), "RealSense (%s) did not receive frames last %d seconds", left_or_right_.c_str(),
            frame_wait_counter_ * (int)frame_timeout_);
    }

    rs2::frameset frames = pipe_.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    // Downsample points -> spatial smoothing for noise -> averaging depth values over time to reduce temporal noise
    depth = dec_filter_.process(depth);
    depth = spat_filter_.process(depth);
    depth = temp_filter_.process(depth);

    // Allow default constructor for pc
    // NOLINTNEXTLINE
    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth);

    // TODO: See if we should change this to some other point cloud structure  
    PointCloud::Ptr pointcloud = points_to_pcl(points);
    pointcloud->header.frame_id = "camera_front_" + left_or_right_ + "_depth_optical_frame";

    processPointCloud(pointcloud);
}


// TODO: Change this to MuJoCo sim

/**
 * Callback function for when a simulated RealSense depth frame arrives.
 *
 * @param input_cloud PointCloud2 message from the realsense MuJoCo plugin
 * callback.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void CamerasInterface::processSimulatedDepthFrames(
    const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud) // NOLINT
{
    PointCloud converted_cloud;
    pcl::fromROSMsg(*input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud = boost::make_shared<PointCloud>(converted_cloud);
    processPointCloud(pointcloud);
}


// TODO: This might be for the registration class
void CamerasInterface::processPointCloud(const PointCloud::Ptr& pointcloud)
{
    while (locked_) {
        rclcpp::sleep_for(
            /*nanoseconds=*/std::chrono::nanoseconds(10000000)); // 10 ms
    }
    locked_ = true;

    last_frame_time_ = std::clock();
    frame_wait_counter_ = 0;

    // TODO: Change this to the point cloud registration step

    // Preprocess point cloud and place pointcloud in aligned toes frame:
    preprocessor_->preprocess(pointcloud);
   
    // TODO: Add a transform method to common reference frame 

    // TODO: Change this to upload the fused point cloud
    // Publish cloud for visualization:
    publishCloud(preprocessed_pointcloud_publisher_, n_, *pointcloud, left_or_right_);

    // Visualization
    if (validatePoint(desired_point_) && validatePoint(start_point_)) {
        // publishSearchRectangle(point_marker_publisher_, n_, desired_point_,
        //     displacements_, left_or_right_); // Cyan
        // publishDesiredPosition(point_marker_publisher_, n_, desired_point_,
        //     left_or_right_); // Green
        // publishRelativeSearchPoint(point_marker_publisher_, n_, start_point_,
        //     left_or_right_); // Purple
    }

    locked_ = false;
}


/**
 * Computes a temporal average of the last X points and removes any outliers,
 * then publishes the average of the points without the outliers.
 *
 * @param new_point New point to compute the temporal average with.
 * @return Point Average point of last n number of final points.
 */
Point CamerasInterface::computeTemporalAveragePoint(const Point& new_point) {

    if (found_points_.size() < sample_size_) {
        found_points_.push_back(new_point);
    } else {
        std::rotate(found_points_.begin(), found_points_.begin() + 1, found_points_.end());
        found_points_[sample_size_ - 1] = new_point;
        Point avg = computeAveragePoint(found_points_);

        std::vector<Point> non_outliers;
        for (Point& p : found_points_) {
            if (pcl::squaredEuclideanDistance(p, avg) < outlier_distance_) {
                non_outliers.push_back(p);
            }
        }

        if (non_outliers.size() == sample_size_) {
            Point final_point = computeAveragePoint(non_outliers);
            return final_point;
        }
    }
}
