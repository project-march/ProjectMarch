/**
 * @author Tuhin Das - MARCH 7
 */

#include "foot_position_finder.h"
#include "point_finder.h"
#include "preprocessor.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities/math_utilities.hpp"
#include "utilities/publish_utilities.hpp"
#include "utilities/realsense_to_pcl.hpp"
#include <iostream>
#include <march_foot_position_finder/parametersConfig.h>
#include <ros/console.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

/**
 * Constructs an object that listens to simulated or real RealSense depth frames
 * and processes these frames with a PointFinder.
 *
 * @param n NodeHandle for running ROS commands
 * @param realsense whether RealSense cameras are connected
 * @param left_or_right whether the FootPositionFinder runs for the left or
 * right foot
 */
// No lint is used to allow uninitialized variables (ros parameters)
// NOLINTNEXTLINE
FootPositionFinder::FootPositionFinder(ros::NodeHandle* n,
    const std::string& left_or_right) // NOLINT
    : n_(n)
    , left_or_right_(left_or_right)
{
    if (left_or_right_ == "left") {
        other_side_ = "right";
        switch_factor_ = -1;
        serial_number_ = "944622074337";
    } else {
        other_side_ = "left";
        switch_factor_ = 1;
        serial_number_ = "944622071535";
    }

    current_frame_id_ = "toes_" + left_or_right_ + "_aligned";
    other_frame_id_ = "toes_" + other_side_ + "_aligned";
    ORIGIN = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    last_height_ = 0;
    refresh_last_height_ = 0;
    last_frame_time_ = std::clock();
    frame_wait_counter_ = 0;
    frame_timeout_ = 5.0;

    topic_camera_front_
        = "/camera_front_" + left_or_right + "/depth/color/points";
    topic_other_chosen_point_
        = "/march/chosen_foot_position/" + other_side_; // in current_frame_id
    topic_current_chosen_point_
        = "/march/chosen_foot_position/" + left_or_right_;

    point_publisher_ = n_->advertise<march_shared_msgs::FootPosition>(
        "/march/foot_position/" + left_or_right_, /*queue_size=*/1);
    preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>(
        "/camera_" + left_or_right_ + "/preprocessed_cloud", /*queue_size=*/1);
    point_marker_publisher_ = n_->advertise<visualization_msgs::Marker>(
        "/camera_" + left_or_right_ + "/found_points", /*queue_size=*/1);

    other_chosen_point_subscriber_
        = n_->subscribe<march_shared_msgs::FootPosition>(
            topic_other_chosen_point_,
            /*queue_size=*/1, &FootPositionFinder::chosenOtherPointCallback,
            this);

    current_state_subscriber_ = n_->subscribe<march_shared_msgs::CurrentState>(
        "/march/gait_selection/current_state", /*queue_size=*/1,
        &FootPositionFinder::currentStateCallback, this);

    height_map_publisher_ = n->advertise<std_msgs::Float64MultiArray>(
        "/debug/height_map", /*queue_size=*/1);

    running_ = false;
}

/**
 * Callback for when parameters are updated with ros reconfigure.
 *
 * @param parametersConfig container that contains all (updated) parameters
 * @param uint32_t level is not used but is required for correct callback
 */
// No lint is used to avoid linting in the RealSense library, where a potential
// memory leak error is present
// NOLINTNEXTLINE
void FootPositionFinder::readParameters(
    march_foot_position_finder::parametersConfig& config, uint32_t level)
{
    foot_gap_ = config.foot_gap;
    step_distance_ = config.step_distance;
    sample_size_ = config.sample_size;
    outlier_distance_ = config.outlier_distance;
    height_zero_threshold_ = config.height_zero_threshold;
    found_points_.resize(sample_size_);
    ros::param::get("/realsense_simulation", realsense_simulation_);

    // Connect the physical RealSense cameras
    if (!running_ && !realsense_simulation_) {
        while (true) {
            try {
                config_.enable_device(serial_number_);
                config_.enable_stream(RS2_STREAM_DEPTH, /*width=*/640,
                    /*height=*/480, RS2_FORMAT_Z16, /*framerate=*/15);
                pipe_.start(config_);
            } catch (const rs2::error& e) {
                std::string error_message = e.what();
                ROS_WARN("Error while initializing %s RealSense camera: %s",
                    left_or_right_.c_str(), error_message.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            realsense_timer_ = n_->createTimer(ros::Duration(/*t=*/0.005),
                &FootPositionFinder::processRealSenseDepthFrames, this);
            ROS_INFO("\033[1;36m%s RealSense connected (%s) \033[0m",
                left_or_right_.c_str(), serial_number_.c_str());

            break;
        }
    }

    // Initialize the callback for the RealSense simulation plugin
    if (!running_ && realsense_simulation_) {
        pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(
            topic_camera_front_, /*queue_size=*/1,
            &FootPositionFinder::processSimulatedDepthFrames, this);
        ROS_INFO(
            "\033[1;36mSimulated RealSense callback initialized (%s) \033[0m",
            left_or_right_.c_str());
    }

    // Initialize all variables as zero:
    last_displacement_ = previous_start_point_ = start_point_
        = transformPoint(ORIGIN, current_frame_id_, other_frame_id_);

    desired_point_ = addPoints(start_point_,
        Point(-(float)step_distance_, (float)(switch_factor_ * foot_gap_),
            /*_z=*/0));

    ROS_INFO("Parameters updated in %s foot position finder",
        left_or_right_.c_str());

    running_ = true;
}

/**
 * Callback function for when the gait selection node selects a point for the
 * other leg.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::chosenOtherPointCallback(
    const march_shared_msgs::FootPosition msg) // NOLINT
{
    // Start point in current frame is equal to the previous displacement:
    last_displacement_ = start_point_
        = Point(msg.displacement.x, msg.displacement.y, msg.displacement.z);

    // previous_start_point_ is the current origin:
    previous_start_point_ = ORIGIN;

    // Compute desired point:
    desired_point_ = addPoints(start_point_,
        Point(-(float)step_distance_, (float)(switch_factor_ * foot_gap_),
            /*_z=*/0));
}

/**
 * Callback function to reset the position values when the exoskeleton enters
 * the "stand" state.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::currentStateCallback(
    const march_shared_msgs::CurrentState msg)
{
    if (msg.state == "stand") {
        initial_position_reset_timer_ = n_->createTimer(
            ros::Duration(/*t=*/0.200),
            &FootPositionFinder::resetInitialPosition, this, /*oneshot=*/true);
    }
}

/**
 * Reset initial position, relative to which points are found.
 */
void FootPositionFinder::resetInitialPosition(const ros::TimerEvent&)
{
    last_displacement_ = previous_start_point_ = start_point_
        = transformPoint(ORIGIN, current_frame_id_, other_frame_id_);
    desired_point_ = addPoints(start_point_,
        Point(-(float)step_distance_, (float)(switch_factor_ * foot_gap_),
            /*_z=*/0));
}

/**
 * Listen for RealSense frames from a camera, apply filters to them and process
 * the eventual pointcloud.
 */
void FootPositionFinder::processRealSenseDepthFrames(const ros::TimerEvent&)
{
    float difference = float(std::clock() - last_frame_time_) / CLOCKS_PER_SEC;
    if ((int)(difference / frame_timeout_) > frame_wait_counter_) {
        frame_wait_counter_++;
        ROS_WARN("RealSense (%s) did not receive frames last %d seconds",
            left_or_right_.c_str(), frame_wait_counter_ * (int)frame_timeout_);
    }

    rs2::frameset frames = pipe_.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    depth = dec_filter_.process(depth);
    depth = spat_filter_.process(depth);
    depth = temp_filter_.process(depth);

    // Allow default constructor for pc
    // NOLINTNEXTLINE
    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth);

    PointCloud::Ptr pointcloud = points_to_pcl(points);
    pointcloud->header.frame_id
        = "camera_front_" + left_or_right_ + "_depth_optical_frame";
    processPointCloud(pointcloud);
}

/**
 * Callback function for when a simulated RealSense depth frame arrives.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::processSimulatedDepthFrames(
    const sensor_msgs::PointCloud2 input_cloud) // NOLINT
{
    PointCloud converted_cloud;
    pcl::fromROSMsg(input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud
        = boost::make_shared<PointCloud>(converted_cloud);
    processPointCloud(pointcloud);
}

/**
 * Run a complete processing pipeline for a point cloud with as a result a new
 * point.
 */
void FootPositionFinder::processPointCloud(const PointCloud::Ptr& pointcloud)
{
    last_frame_time_ = std::clock();
    frame_wait_counter_ = 0;

    // Preprocess point cloud and place pointcloud in aligned toes frame:
    Preprocessor preprocessor(pointcloud, left_or_right_, listener);
    preprocessor.preprocess();

    // Publish cloud for visualization:
    publishCloud(
        preprocessed_pointcloud_publisher_, *pointcloud, left_or_right_);

    // Find possible points around the desired point determined earlier:
    PointFinder pointFinder(
        n_, pointcloud, left_or_right_, desired_point_, height_map_publisher_);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    // Visualization
    publishSearchRectangle(point_marker_publisher_, desired_point_,
        pointFinder.getDisplacements(), left_or_right_);
    publishDesiredPosition(
        point_marker_publisher_, desired_point_, left_or_right_); // Green
    publishRelativeSearchPoint(point_marker_publisher_, start_point_,
        left_or_right_); // Purple

    if (position_queue.size() > 0) {
        // Take the first point of the point queue returned by the point finder
        found_covid_point_ = computeTemporalAveragePoint(position_queue[0]);

        // Retrieve 3D points between current and new determined foot position
        // previous_start_point_ is where the current leg is right now
        std::vector<Point> track_points = pointFinder.retrieveTrackPoints(
            previous_start_point_, found_covid_point_);

        // Visualization
        publishTrackMarkerPoints(
            point_marker_publisher_, track_points, left_or_right_); // Orange
        publishMarkerPoint(
            point_marker_publisher_, found_covid_point_, left_or_right_); // Red
        publishPossiblePoints(
            point_marker_publisher_, position_queue, left_or_right_);

        // Compute new foot displacement for gait computation
        new_displacement_ = subtractPoints(found_covid_point_, start_point_);

        // Apply a threshold for the height of points to be different from 0
        if (std::abs(new_displacement_.z) < height_zero_threshold_) {
            new_displacement_.z = 0.0;
        }

        // Visualization
        publishArrow(point_marker_publisher_, ORIGIN, start_point_,
            left_or_right_); // Blue
        publishArrow2(point_marker_publisher_, start_point_, found_covid_point_,
            left_or_right_); // Green

        // Publish final point for gait computation
        publishPoint(point_publisher_, found_covid_point_, found_covid_point_,
            new_displacement_, track_points);
    }
}

/**
 * Computes a temporal average of the last X points and removes any outliers,
 * then publishes the average of the points without the outliers.
 */
Point FootPositionFinder::computeTemporalAveragePoint(const Point& new_point)
{
    if (found_points_.size() < sample_size_) {
        found_points_.push_back(new_point);
    } else {
        std::rotate(found_points_.begin(), found_points_.begin() + 1,
            found_points_.end());
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

/**
 * Transforms a point in place from one frame to another using ROS
 * transformations
 *
 * @param point Point to transform between frames
 * @param frame_from source frame in which the point is currently
 * @param frame_to target frame in which point is transformed
 */
Point FootPositionFinder::transformPoint(
    Point point, const std::string& frame_from, const std::string& frame_to)
{
    PointCloud::Ptr desired_point = boost::make_shared<PointCloud>();
    desired_point->push_back(point);

    tf::StampedTransform transform_stamped;

    try {
        ros::Time now = ros::Time::now();
        listener.waitForTransform(
            frame_from, frame_to, now, ros::Duration(/*t=*/1.0));
        listener.lookupTransform(frame_from, frame_to, now, transform_stamped);

    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM(
            "Something went wrong when transforming the pointcloud: "
            << ex.what());
    }

    pcl_ros::transformPointCloud(
        *desired_point, *desired_point, transform_stamped);

    return desired_point->points[0];
}
