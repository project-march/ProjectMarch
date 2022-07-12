/**
 * @author Tuhin Das - MARCH 7
 */

#include "foot_position_finder.h"
#include "point_finder.h"
#include "preprocessor.h"
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
    base_frame_ = "world";
    ORIGIN = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    last_height_ = 0;
    refresh_last_height_ = 0;
    last_frame_time_ = std::clock();
    frame_wait_counter_ = 0;
    frame_timeout_ = 5.0;

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
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

    current_chosen_point_subscriber_
        = n_->subscribe<march_shared_msgs::FootPosition>(
            topic_current_chosen_point_,
            /*queue_size=*/1, &FootPositionFinder::chosenCurrentPointCallback,
            this);

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
    base_frame_ = config.base_frame;
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

    // Initialize position of other foot in current frame
    // This position is equal to the last displacement in current frame
    start_point_current_ = last_displacement_
        = transformPoint(ORIGIN, other_frame_id_, current_frame_id_);

    // Initialize position of other foot in base frame
    last_chosen_point_world_
        = transformPoint(start_point_current_, current_frame_id_, base_frame_);

    // The last height is used to remember how high the previous step was of the
    // other foot (relative to the hip base). Here is it initialized to the zero
    // point in the base frame
    Point height_init = transformPoint(ORIGIN, base_frame_, "hip_base_aligned");
    last_height_ = height_init.z;

    // Current start point in world frame (for visualization)
    start_point_world_
        = transformPoint(start_point_current_, current_frame_id_, base_frame_);
    // The previous point of the current foot (for visualization)
    previous_start_point_world_
        = transformPoint(ORIGIN, current_frame_id_, base_frame_);

    // Desired point = (current start point) + (usual displacement)
    // The displacement is the vector (-step_distance_, +-foot_gap_, 0)
    desired_point_world_ = addPoints(start_point_current_,
        Point(-(float)step_distance_, (float)(switch_factor_ * foot_gap_),
            /*_z=*/0));
    // Rotation necessary for base_frame computation
    desired_point_world_ = rotateRight(
        transformPoint(desired_point_world_, current_frame_id_, base_frame_));

    ROS_INFO("Parameters updated in %s foot position finder",
        left_or_right_.c_str());

    running_ = true;
}

/**
 * Callback function for when the gait selection node selects a point for the
 * current leg.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::chosenCurrentPointCallback(
    const march_shared_msgs::FootPosition msg) // NOLINT
{
    // Schedule when last_height_ is updated with the height of the other leg.
    // This timer is used to simulate the moment when pressure soles indicate a
    // touch of the ground. Currently the duration is equal to the early
    // schedule duration, but this should be loaded dynamically eventually.
    height_reset_timer_ = n_->createTimer(ros::Duration(/*t=*/0.250),
        &FootPositionFinder::resetHeight, this, /*oneshot=*/true);
}

/**
 * Callback function for when the gait selection node selects a point for the
 * other leg.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::chosenOtherPointCallback(
    const march_shared_msgs::FootPosition msg) // NOLINT
{
    // Start point in current frame is equal to the previous displacement
    last_displacement_ = start_point_current_
        = Point(msg.displacement.x, msg.displacement.y, msg.displacement.z);
    // Store previous chosen point of other foot in world frame
    start_point_world_
        = Point(msg.point_world.x, msg.point_world.y, msg.point_world.z);

    // previous_start_point_world_ is the previous start point (for
    // visualization)
    previous_start_point_world_
        = transformPoint(ORIGIN, current_frame_id_, base_frame_);

    // Compute desired point in base_frame_
    desired_point_world_ = addPoints(start_point_current_,
        Point(-(float)step_distance_, (float)(switch_factor_ * foot_gap_),
            /*_z=*/0));
    // Rotation is necessary for visualization and computation in base frame
    desired_point_world_ = rotateRight(
        transformPoint(desired_point_world_, current_frame_id_, base_frame_));
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
 * Reset last height when other leg lands on the ground; the height is
 * determined relative to the hip base.
 */
void FootPositionFinder::resetHeight(const ros::TimerEvent&)
{
    Point reset_zero
        = transformPoint(ORIGIN, other_frame_id_, "hip_base_aligned");
    last_height_ = reset_zero.z;
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

    // Preprocess point cloud
    NormalCloud::Ptr normalcloud(new NormalCloud());
    Preprocessor preprocessor(n_, pointcloud, normalcloud);
    preprocessor.preprocess();

    // Publish cloud for visualization
    publishCloud(preprocessed_pointcloud_publisher_, *pointcloud);

    // Find possible points around the desired point determined earlier
    PointFinder pointFinder(
        n_, pointcloud, left_or_right_, desired_point_world_);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    // Visualization
    publishSearchRectangle(point_marker_publisher_, desired_point_world_,
        pointFinder.getDisplacements(), left_or_right_);
    publishDesiredPosition(
        point_marker_publisher_, desired_point_world_); // Green
    publishRelativeSearchPoint(point_marker_publisher_,
        rotateRight(start_point_world_)); // Purple

    if (position_queue.size() > 0) {
        // Take the first point of the point queue returned by the point finder
        Point found_covid_point_world
            = computeTemporalAveragePoint(position_queue[0]); // Red

        // Retrieve 3D points between current and new determined foot position
        // previous_start_point_ is where the current leg is right now
        std::vector<Point> track_points = pointFinder.retrieveTrackPoints(
            rotateRight(previous_start_point_world_), found_covid_point_world);

        // Visualization
        publishTrackMarkerPoints(point_marker_publisher_, track_points);
        publishMarkerPoint(point_marker_publisher_, found_covid_point_world);
        publishPossiblePoints(point_marker_publisher_, position_queue);

        // Transform point found with cameras to current frame and to hip frame
        found_covid_point_world = rotateLeft(found_covid_point_world);
        Point found_covid_point_current_ = transformPoint(
            found_covid_point_world, base_frame_, current_frame_id_);
        Point found_covid_point_hip_ = transformPoint(
            found_covid_point_world, base_frame_, "hip_base_aligned");

        // Compute new foot displacement for gait computation
        Point new_displacement
            = subtractPoints(found_covid_point_current_, start_point_current_);
        // Compute the z displacement with the z-values in hip frame.
        float displacement_z = found_covid_point_hip_.z - last_height_;
        new_displacement.z = displacement_z;

        // Apply a threshold for the height of points to be different from 0
        if (std::abs(new_displacement.z) < height_zero_threshold_) {
            new_displacement.z = 0;
        }

        // Transform the height points between start and end position to current
        // frame
        std::vector<Point> relative_track_points;
        for (Point& p : track_points) {
            Point point = rotateLeft(p);
            point = transformPoint(point, base_frame_, current_frame_id_);
            relative_track_points.emplace_back(point);
        }

        // Visualization
        publishArrow(point_marker_publisher_, previous_start_point_world_,
            start_point_world_); // Blue
        publishArrow2(point_marker_publisher_, start_point_world_,
            found_covid_point_world);

        // Publish final point for gait computation
        publishPoint(point_publisher_, found_covid_point_current_,
            found_covid_point_world, new_displacement, relative_track_points);
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

    geometry_msgs::TransformStamped transform_stamped;

    try {
        if (tfBuffer_->canTransform(frame_to, frame_from, ros::Time(/*t=*/0),
                ros::Duration(/*t=*/1.0))) {
            transform_stamped = tfBuffer_->lookupTransform(
                frame_to, frame_from, ros::Time(/*t=*/0));
        }
    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM(
            "Something went wrong when transforming the pointcloud: "
            << ex.what());
    }

    pcl_ros::transformPointCloud(
        *desired_point, *desired_point, transform_stamped.transform);

    return desired_point->points[0];
}
