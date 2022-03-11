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
 * Constructs an object that listens to simulated or real realsense depth frames
 * and processes these frames with a PointFinder.
 *
 * @param n NodeHandle for running ROS commands
 * @param realsense whether realsense cameras are connected
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
        other_ = "right";
        switch_factor_ = 1;
    } else {
        switch_factor_ = -1;
        other_ = "left";
    }

    reference_frame_id_ = "foot_" + other_;
    current_frame_id_ = "foot_" + left_or_right;


    last_chosen_point_world_ = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
    topic_camera_front_
        = "/camera_front_" + left_or_right + "/depth/color/points";
    topic_chosen_point_
        = "/chosen_foot_position/" + other_; // in current_frame_id

    point_publisher_ = n_->advertise<march_shared_msgs::FootPosition>(
        "/foot_position/" + left_or_right_, /*queue_size=*/1);
    preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>(
        "/camera_" + left_or_right_ + "/preprocessed_cloud", /*queue_size=*/1);
    point_marker_publisher_ = n_->advertise<visualization_msgs::Marker>(
        "/camera_" + left_or_right_ + "/found_points", /*queue_size=*/1);

    chosen_point_subscriber_
        = n_->subscribe<march_shared_msgs::FootPosition>(topic_chosen_point_,
            /*queue_size=*/1, &FootPositionFinder::chosenPointCallback, this);

    running_ = false;
}

/**
 * Callback for when parameters are updated with ros reconfigure.
 */
// No lint is used to avoid linting in the realsense library, where a potential
// memory leak error is present
// NOLINTNEXTLINE
void FootPositionFinder::readParameters(
    march_foot_position_finder::parametersConfig& config, uint32_t level)
{
    physical_cameras_ = config.physical_cameras;
    base_frame_ = config.base_frame;
    base_frame_ = "world";
    foot_gap_ = config.foot_gap;
    step_distance_ = config.step_distance;
    sample_size_ = config.sample_size;
    outlier_distance_ = config.outlier_distance;
    found_points_.resize(sample_size_);

    // Initialize the depth frame callbacks the first time parameters are read
    if (!running_ && physical_cameras_) {
        config_.enable_stream(RS2_STREAM_DEPTH, /*width=*/640, /*height=*/480,
            RS2_FORMAT_Z16, /*framerate=*/30);

        if (left_or_right_ == "left") {
            config_.enable_device("944622074337");
        } else {
            config_.enable_device("944622071535");
        }

        pipe_.start(config_);
        ROS_INFO("Realsense camera (%s) connected", left_or_right_.c_str());

        realsenseTimer = n_->createTimer(ros::Duration(/*t=*/0.005),
            &FootPositionFinder::processRealSenseDepthFrames, this);

    } else if (!running_ && !physical_cameras_) {
        pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(
            topic_camera_front_, /*queue_size=*/1,
            &FootPositionFinder::processSimulatedDepthFrames, this);
    }

    Point start(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    // last_chosen_point_ = transformPoint(start, reference_frame_id_, base_frame_);
    last_chosen_point_ = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    last_chosen_point_ = transformPoint(last_chosen_point_, reference_frame_id_, current_frame_id_);
    last_chosen_point_visualize_ = transformPoint(last_chosen_point_, current_frame_id_, "world");

    last_chosen_point_world_ = transformPoint(start, reference_frame_id_, base_frame_);
        
    if (left_or_right_ == "left") {
        last_displacement_ = Point(/*_x=*/0, /*_y=*/(float)foot_gap_, /*_z=*/0);
    } else {
        last_displacement_
            = Point(/*_x=*/0, /*_y=*/-(float)foot_gap_, /*_z=*/0);
    }

    last_chosen_point_ = last_displacement_;

    ROS_INFO("Parameters updated in %s foot position finder",
        left_or_right_.c_str());

    running_ = true;
}

/**
 * Callback function for when the gait selection node selects a point.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::chosenPointCallback(
    const march_shared_msgs::FootPosition msg) // NOLINT
{
    last_chosen_point_ = Point(msg.point.x, msg.point.y, msg.point.z);
    last_chosen_point_world_
        = Point(msg.point_world.x, msg.point_world.y, msg.point_world.z);
    last_displacement_
        = Point(msg.displacement.x, msg.displacement.y, msg.displacement.z);

    last_chosen_point_ = last_displacement_;
    // last_chosen_point_ = transformPoint(last_chosen_point_, reference_frame_id_, current_frame_id_);
    last_chosen_point_visualize_ = transformPoint(last_chosen_point_, current_frame_id_, "world");

}

/**
 * Listen for realsense frames from a camera, apply filters to them and process
 * the eventual pointcloud.
 */
void FootPositionFinder::processRealSenseDepthFrames(const ros::TimerEvent&)
{
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
 * Callback function for when a simulated realsense depth frame arrives.
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
    NormalCloud::Ptr normalcloud(new NormalCloud());
    Preprocessor preprocessor(n_, pointcloud, normalcloud);
    preprocessor.preprocess();
    // Publish cloud for visualization
    publishCloud(preprocessed_pointcloud_publisher_, *pointcloud);

    Point start_point = last_chosen_point_;
    // Point start_point = transformPoint(last_chosen_point_, reference_frame_id_, current_frame_id_);
    Point point;

    if (left_or_right_ == "left") {
        point = Point(start_point.x - (float)step_distance_,
            start_point.y - (float)foot_gap_, start_point.z);
    } else {
        point = Point(start_point.x - (float)step_distance_,
            start_point.y + (float)foot_gap_, start_point.z);
    }

    // Visualize
    // Point p2 = transformPoint(start_point, current_frame_id_, "world");
    // p2 = Point(start_point.y, -start_point.x, start_point.z); // Rotate right
    Point p2 = Point(last_chosen_point_visualize_.y, -last_chosen_point_visualize_.x, last_chosen_point_visualize_.z); // Rotate right
    publishRelativeSearchPoint(point_marker_publisher_, p2);

    point = transformPoint(point, current_frame_id_, "world");
    Point position(point.y, -point.x, point.z); // Rotate right
    publishDesiredPosition(point_marker_publisher_, position);

    
    PointFinder pointFinder(n_, pointcloud, left_or_right_, position);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    // Publish search region for visualization
    publishSearchRectangle(point_marker_publisher_, position,
        pointFinder.getDisplacements(), left_or_right_);

    if (position_queue.size() > 0) {
        Point avg = computeTemporalAveragePoint(position_queue[0]);

        // Retrieve 3D points between current and new foot position
        Point start(/*_x=*/0, /*_y=*/0, /*_z=*/0);
        start = transformPoint(start, current_frame_id_, base_frame_);
        start = Point(start.y, -start.x, start.z); // Rotate right
        std::vector<Point> track_points
            = pointFinder.retrieveTrackPoints(start, avg);

        // Publish for visualization
        publishTrackMarkerPoints(point_marker_publisher_, track_points);
        publishMarkerPoint(point_marker_publisher_, avg);
        publishPossiblePoints(point_marker_publisher_, position_queue);

        Point current_frame_avg = Point(-avg.y, avg.x, avg.z); // Rotate left
        // the desired displacement? -> blue
        publishArrow(point_marker_publisher_, last_chosen_point_visualize_, current_frame_avg);

        Point pp1 = Point(0, 0, 0); 
        Point pp2 = last_displacement_;
        pp1 = transformPoint(pp1, current_frame_id_, "world");
        pp2 = transformPoint(pp2, current_frame_id_, "world");
        // the displacement given by the gait callback -> green
        publishArrow2(point_marker_publisher_, pp1, pp2);

        current_frame_avg
            = transformPoint(current_frame_avg, "world", current_frame_id_);

        std::vector<Point> relative_track_points;
        // for (Point& p : track_points) {
        //     Point point(-p.y, p.x, p.z); // Rotate left
        //     point = transformPoint(point, base_frame_, current_frame_id_);
        //     relative_track_points.emplace_back(point);
        // }

        // std::cout << left_or_right_ << std::endl;
        // std::cout << world_frame_avg << std::endl;
        // std::cout << start_point.z << std::endl;

        Point displacement(current_frame_avg.x - start_point.x,
            current_frame_avg.y - start_point.y,
            current_frame_avg.z - start_point.z);


        Point world_frame_avg = Point(-avg.y, avg.x, avg.z);
        publishPoint(point_publisher_, current_frame_avg, world_frame_avg,
            displacement, relative_track_points);
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
    Point& point, const std::string& frame_from, const std::string& frame_to)
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
