/**
 * @author Tuhin Das - MARCH 7
 */

#include "foot_position_finder.h"
#include "utilities/math_utilities.hpp"
#include "utilities/publish_utilities.hpp"
#include "utilities/realsense_to_pcl.hpp"
#include <iostream>
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/msg/marker_array.hpp>

/**
 * Constructs an object that listens to simulated or real RealSense depth frames
 * and processes these frames with a PointFinder.
 *
 * @param n ROS Node instance.
 * @param left_or_right whether the FootPositionFinder runs for the left or
 * right foot.
 */
// No lint is used to allow uninitialized variables (ros parameters)
// NOLINTNEXTLINE
FootPositionFinder::FootPositionFinder(rclcpp::Node* n,
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

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(n_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    preprocessor_ = std::make_unique<Preprocessor>(
        n_, left_or_right_, tf_listener_, tf_buffer_);
    point_finder_ = std::make_unique<PointFinder>(n_, left_or_right_);

    current_frame_id_ = "toes_" + left_or_right_ + "_aligned";
    other_frame_id_ = "toes_" + other_side_ + "_aligned";
    ORIGIN = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    last_height_ = 0;
    refresh_last_height_ = 0;
    last_frame_time_ = std::clock();
    frame_wait_counter_ = 0;
    frame_timeout_ = 5.0;
    paused_ = false;

    topic_camera_front_
        = "/camera_front_" + left_or_right + "/depth/color/points";
    topic_other_chosen_point_
        = "/march/chosen_foot_position/" + other_side_; // in current_frame_id
    topic_current_chosen_point_
        = "/march/chosen_foot_position/" + left_or_right_;

    point_publisher_
        = n_->create_publisher<march_shared_msgs::msg::FootPosition>(
            "/march/foot_position/" + left_or_right_, /*qos=*/1);
    preprocessed_pointcloud_publisher_
        = n_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera_" + left_or_right_ + "/preprocessed_cloud",
            /*qos=*/1);
    point_marker_publisher_
        = n_->create_publisher<visualization_msgs::msg::Marker>(
            "/camera_" + left_or_right_ + "/found_points", /*qos=*/1);

    other_chosen_point_subscriber_
        = n_->create_subscription<march_shared_msgs::msg::FootPosition>(
            topic_other_chosen_point_,
            /*qos=*/1,
            std::bind(&FootPositionFinder::chosenOtherPointCallback, this,
                std::placeholders::_1));

    std::function<void(
        const march_shared_msgs::msg::FootPosition::SharedPtr msg)>
        point_callback
        = std::bind(&FootPositionFinder::chosenOtherPointCallback, this,
            std::placeholders::_1);
    other_chosen_point_subscriber_
        = n_->create_subscription<march_shared_msgs::msg::FootPosition>(
            topic_other_chosen_point_,
            /*qos=*/1, point_callback);

    std::function<void(
        const march_shared_msgs::msg::CurrentState::SharedPtr msg)>
        state_callback = std::bind(&FootPositionFinder::currentStateCallback,
            this, std::placeholders::_1);
    current_state_subscriber_
        = n_->create_subscription<march_shared_msgs::msg::CurrentState>(
            "/march/gait_selection/current_state",
            /*qos=*/1, state_callback);

    foot_gap_ = n_->get_parameter("foot_gap").as_double();
    step_distance_ = n_->get_parameter("step_distance").as_double();
    sample_size_ = n_->get_parameter("sample_size").as_int();

    outlier_distance_ = n_->get_parameter("outlier_distance").as_double();
    height_zero_threshold_
        = n_->get_parameter("height_zero_threshold").as_double();
    realsense_simulation_ = n_->get_parameter("realsense_simulation").as_bool();
    found_points_.resize(sample_size_);
    displacements_ = point_finder_->getDisplacements();

    // Connect the physical RealSense cameras
    if (!realsense_simulation_) {
        while (true) {
            try {
                config_.enable_device(serial_number_);
                config_.enable_stream(RS2_STREAM_DEPTH, /*width=*/640,
                    /*height=*/480, RS2_FORMAT_Z16, /*framerate=*/15);
                pipe_.start(config_);
            } catch (const rs2::error& e) {
                std::string error_message = e.what();
                RCLCPP_WARN(n_->get_logger(),
                    "Error while initializing %s RealSense camera: %s",
                    left_or_right_.c_str(), error_message.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            realsense_timer_ = n_->create_wall_timer(
                std::chrono::milliseconds(5), [this]() -> void {
                    processRealSenseDepthFrames();
                });

            RCLCPP_INFO(n_->get_logger(),
                "\033[1;36m%s RealSense connected (%s) \033[0m",
                left_or_right_.c_str(), serial_number_.c_str());

            break;
        }
    } else {
        // Initialize the callback for the RealSense simulation plugin
        std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)>
            callback
            = std::bind(&FootPositionFinder::processSimulatedDepthFrames, this,
                std::placeholders::_1);
        pointcloud_subscriber_
            = n_->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_camera_front_,
                /*qos=*/1, callback);

        RCLCPP_INFO(n_->get_logger(),
            "\033[1;36mSimulated RealSense callback initialized (%s)\033[0m",
            left_or_right_.c_str());
    }

    resetInitialPosition(/*stop_timer=*/false);
}

/**
 * Retrieve parameter values that are dynamically reconfigured and update the
 * values in the class.
 *
 * @param parameters Instance containing updated parameters.
 */
void FootPositionFinder::readParameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
    paused_ = true;
    for (const auto& param : parameters) {
        if (param.get_name() == "foot_gap") {
            foot_gap_ = param.as_double();
        } else if (param.get_name() == "step_distance") {
            step_distance_ = param.as_double();
        } else if (param.get_name() == "sample_size") {
            sample_size_ = param.as_int();
        } else if (param.get_name() == "outlier_distance") {
            outlier_distance_ = param.as_double();
        } else if (param.get_name() == "height_zero_threshold") {
            height_zero_threshold_ = param.as_double();
        } else if (param.get_name() == "realsense_simulation") {
            realsense_simulation_ = param.as_bool();
        }

        RCLCPP_INFO(n_->get_logger(),
        "\033[92mParameter %s updated in %s Foot Position Finder\033[0m",
        param.get_name().c_str(), left_or_right_.c_str());
    }

    found_points_.resize(sample_size_);
    resetInitialPosition(/*stop_timer=*/false);
    point_finder_->readParameters(parameters);
    displacements_ = point_finder_->getDisplacements();
    paused_ = false;

    RCLCPP_INFO(n_->get_logger(), "\033[92mUpdate finished\033[0m");
}

/**
 * Callback function for when the gait selection node selects a point for the
 * other leg.
 *
 * @param msg FootPosition message from the callback.
 */
// Suppress lint error "make reference of argument" as it breaks callback
void FootPositionFinder::chosenOtherPointCallback(
    const march_shared_msgs::msg::FootPosition::SharedPtr msg) // NOLINT
{
    // Start point in current frame is equal to the previous displacement:
    start_point_
        = Point(msg->displacement.x, msg->displacement.y, msg->displacement.z);

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
 *
 * @param msg CurrentState message from the callback.
 */
// Suppress lint error "make reference of argument" as it breaks the callback
void FootPositionFinder::currentStateCallback(
    const march_shared_msgs::msg::CurrentState::SharedPtr msg) // NOLINT
{
    if (msg->state == "stand") {
        initial_position_reset_timer_ = n_->create_wall_timer(
            std::chrono::milliseconds(200), [this]() -> void {
                resetInitialPosition(/*stop_timer=*/true);
            });
    }
}

/**
 * Reset initial position, relative to which points are found.
 *
 * @param stop_timer Whether to stop a ROS timer that scheduled this callback.
 */
void FootPositionFinder::resetInitialPosition(bool stop_timer)
{
    previous_start_point_ = start_point_
        = transformPoint(ORIGIN, other_frame_id_, current_frame_id_);
    desired_point_ = addPoints(start_point_,
        Point(-(float)step_distance_, (float)(switch_factor_ * foot_gap_),
            /*_z=*/0));

    if (stop_timer) {
        initial_position_reset_timer_->cancel();
    }
}

/**
 * Listen for RealSense frames from a camera, apply filters to them and process
 * the eventual pointcloud.
 */
void FootPositionFinder::processRealSenseDepthFrames()
{
    float difference = float(std::clock() - last_frame_time_) / CLOCKS_PER_SEC;
    if ((int)(difference / frame_timeout_) > frame_wait_counter_) {
        frame_wait_counter_++;
        RCLCPP_WARN(n_->get_logger(),
            "RealSense (%s) did not receive frames last %d seconds",
            left_or_right_.c_str(), frame_wait_counter_ * (int)frame_timeout_);
    }

    if (paused_) {
        return;
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
 *
 * @param input_cloud PointCloud2 message from the realsense gazebo plugin
 * callback.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::processSimulatedDepthFrames(
    const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud) // NOLINT
{
    PointCloud converted_cloud;
    pcl::fromROSMsg(*input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud
        = boost::make_shared<PointCloud>(converted_cloud);
    processPointCloud(pointcloud);
}

/**
 * Run a complete processing pipeline for a point cloud with as a result a new
 * point.
 *
 * @param pointcloud Pointcloud to find points in.
 */
void FootPositionFinder::processPointCloud(const PointCloud::Ptr& pointcloud)
{
    if (paused_) {
        return;
    }

    last_frame_time_ = std::clock();
    frame_wait_counter_ = 0;

    // Preprocess point cloud and place pointcloud in aligned toes frame:
    preprocessor_->preprocess(pointcloud);

    // Publish cloud for visualization:
    publishCloud(
        preprocessed_pointcloud_publisher_, n_, *pointcloud, left_or_right_);

    // Find possible points around the desired point determined earlier:
    std::vector<Point> position_queue;
    point_finder_->findPoints(pointcloud, desired_point_, &position_queue);

    // Visualization
    if (validatePoint(desired_point_) && validatePoint(start_point_)) {
        publishSearchRectangle(point_marker_publisher_, n_, desired_point_,
            displacements_, left_or_right_); // Cyan
        publishDesiredPosition(
            point_marker_publisher_, n_, desired_point_, left_or_right_); // Green
        publishRelativeSearchPoint(point_marker_publisher_, n_, start_point_,
            left_or_right_); // Purple
        publishPreviousDisplacement(point_marker_publisher_, n_, ORIGIN,
            start_point_,
            left_or_right_); // Blue
    }

    if (position_queue.size() > 0) {
        Point optimal_point = retrieveOptimalPoint(&position_queue);

        // Take the first point of the point queue returned by the point finder
        found_covid_point_ = computeTemporalAveragePoint(optimal_point);

        // Retrieve 3D points between current and new determined foot position
        // previous_start_point_ is where the current leg is right now
        std::vector<Point> track_points
            = point_finder_->retrieveTrackPoints(ORIGIN, found_covid_point_);

        // Visualization
        

        // Compute new foot displacement for gait computation
        new_displacement_ = subtractPoints(found_covid_point_, start_point_);

        // Apply a threshold for the height of points to be different from 0
        if (std::abs(new_displacement_.z) < height_zero_threshold_) {
            new_displacement_.z = 0.0;
        }

        // Visualize new displacement
        publishTrackMarkerPoints(point_marker_publisher_, n_, track_points,
            left_or_right_); // Orange
        publishMarkerPoint(point_marker_publisher_, n_, found_covid_point_,
            left_or_right_); // Red
        publishPossiblePoints(
            point_marker_publisher_, n_, position_queue, left_or_right_);
        publishNewDisplacement(point_marker_publisher_, n_, start_point_,
            found_covid_point_,
            left_or_right_); // Green

        // Publish final point for gait computation
        publishPoint(point_publisher_, n_, found_covid_point_,
            found_covid_point_, new_displacement_, track_points);
    }
}

/**
 * Compute the optimal step point by maximizing the value `distance - factor *
 * abs(height)`. This results in finding the furthest point that has the least
 * height difference.
 *
 * @param position_queue Queue with possible foot positions.
 * @return Point Final optimal point to step towards.
 */
Point FootPositionFinder::retrieveOptimalPoint(
    std::vector<Point>* position_queue)
{
    Point optimal_point = *position_queue->begin();
    double optimal_distance_height_tradeoff = 0;

    for (auto p = position_queue->begin(); p != position_queue->end(); ++p) {
        double new_tradeoff = std::abs(p->x) - 2.5 * std::abs(p->z);
        if (new_tradeoff > optimal_distance_height_tradeoff) {
            optimal_point = (*p);
            optimal_distance_height_tradeoff = new_tradeoff;
        }
    }

    return optimal_point;
}

/**
 * Computes a temporal average of the last X points and removes any outliers,
 * then publishes the average of the points without the outliers.
 *
 * @param new_point New point to compute the temporal average with.
 * @return Point Average point of last n number of final points.
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
 * transformations.
 *
 * @param point Point to transform between frames.
 * @param frame_from Source frame in which the point is currently.
 * @param frame_to Target frame in which point is transformed.
 * @return Point Transformed point.
 */
Point FootPositionFinder::transformPoint(
    Point point, const std::string& frame_from, const std::string& frame_to)
{
    PointCloud::Ptr desired_point = boost::make_shared<PointCloud>();
    desired_point->push_back(point);

    geometry_msgs::msg::TransformStamped transform_;
    try {
        transform_ = tf_buffer_->lookupTransform(
            frame_to, frame_from, tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(n_->get_logger(), steady_clock, 4000,
            "Could not transform pointcloud: %s", ex.what());
    }

    Eigen::Matrix<double, 3, 1> translation;
    Eigen::Quaternion<double> rotation;

    tf2::fromMsg(transform_.transform.translation, translation);
    tf2::fromMsg(transform_.transform.rotation, rotation);

    pcl::transformPointCloud(
        *desired_point, *desired_point, translation, rotation);
    desired_point->header.frame_id = frame_to;

    return desired_point->points[0];
}
