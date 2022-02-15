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
// No lint is used to avoid linting in the realsense library, where a potential
// memory leak error is present
// NOLINTNEXTLINE
FootPositionFinder::FootPositionFinder(ros::NodeHandle* n,
    const std::string& left_or_right) // NOLINT
    : n_(n)
    , left_or_right_(left_or_right)
{

    if (left_or_right_ == "left") {
        other_ = "right";
        switch_factor_ = -1;
    } else {
        switch_factor_ = 1;
        other_ = "left";
    }

    reference_frame_id_ = "foot_" + other_;
    current_frame_id_ = "foot_" + left_or_right;
    last_height_ = 0;

    ros::param::get("~realsense", realsense_);
    ros::param::get("~base_frame", base_frame_);
    ros::param::get("~foot_gap", foot_gap_);
    ros::param::get("~step_distance", step_distance_);
    ros::param::get("~average_sample_size", sample_size_);
    ros::param::get("~outlier_distance", outlier_distance_);

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
    topic_camera_front_
        = "/camera_front_" + left_or_right + "/depth/color/points";

    point_publisher_ = n_->advertise<march_shared_msgs::FootPosition>(
        "/foot_position/" + left_or_right_, /*queue_size=*/1);
    preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>(
        "/camera_" + left_or_right_ + "/preprocessed_cloud", /*queue_size=*/1);
    point_marker_publisher_ = n_->advertise<visualization_msgs::Marker>(
        "/camera_" + left_or_right_ + "/found_points", /*queue_size=*/1);

    pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(
        topic_camera_front_, /*queue_size=*/1,
        &FootPositionFinder::processSimulatedDepthFrames, this);

    if (realsense_) {
        // cfg_.enable_stream(RS2_STREAM_DEPTH, /*width=*/640, /*height=*/480,
        //     RS2_FORMAT_Z16, /*framerate=*/30);
        // pipe_.start(cfg_);
        // processRealSenseDepthFrames();
    } else {
        pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(
            topic_camera_front_, /*queue_size=*/1,
            &FootPositionFinder::processSimulatedDepthFrames, this);
    }
}

void FootPositionFinder::readParameters(
    march_foot_position_finder::parametersConfig& config, uint32_t level)
{
    realsense_ = config.realsense;
    base_frame_ = config.base_frame;
    foot_gap_ = config.foot_gap;
    step_distance_ = config.step_distance;
    sample_size_ = config.sample_size;
    outlier_distance_ = config.outlier_distance;
    found_points_.resize(sample_size_);
    ROS_INFO("Parameters updated in foot position finder");
}

/**
 * Listen for realsense frames from a camera, apply filters to them and process
 * the eventual pointcloud.
 */
void FootPositionFinder::processRealSenseDepthFrames()
{
    while (ros::ok()) {
        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();

        depth = dec_filter_.process(depth);
        depth = spat_filter_.process(depth);
        depth = temp_filter_.process(depth);

        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);

        PointCloud::Ptr pointcloud = points_to_pcl(points);
        processPointCloud(pointcloud);
    }
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
    Point point;

    if (!realsense_) {
        // Define the desired future foot position
        point = Point(-(float)step_distance_,
            (float)switch_factor_ * (float)foot_gap_, (float)last_height_);

        // Calculate point location relative to positionary leg
        point = transformPoint(point, reference_frame_id_, base_frame_);
    }

    Preprocessor preprocessor(n_, pointcloud, normalcloud);
    preprocessor.preprocess();

    // Publish cloud for visualization
    publishCloud(preprocessed_pointcloud_publisher_, *pointcloud);

    Point position(point.y, -point.x, point.z);
    PointFinder pointFinder(n_, pointcloud, left_or_right_, position);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    // Publish search region for visualization
    publishSearchRectangle(point_marker_publisher_, position,
        pointFinder.getDisplacements(), left_or_right_);

    if (position_queue.size() > 0) {
        Point avg = computeTemporalAveragePoint(position_queue[0]);

        last_height_ = avg.z;

        // Retrieve 3D points between current and new foot position
        Point start(/*_x=*/0, /*_y=*/0, /*_z=*/0);
        start = transformPoint(start, current_frame_id_, base_frame_);
        start = Point(start.y, -start.x, start.z);
        std::vector<Point> track_points
            = pointFinder.retrieveTrackPoints(start, avg);

        // Publish for visualization
        publishTrackMarkerPoints(point_marker_publisher_, track_points);
        publishMarkerPoint(point_marker_publisher_, avg);
        publishPossiblePoints(point_marker_publisher_, position_queue);

        // Publish new point and points on the track for gait computation
        Point relative_avg = Point(-avg.y, avg.x, avg.z);
        relative_avg
            = transformPoint(relative_avg, base_frame_, reference_frame_id_);
        std::vector<Point> relative_track_points;

        for (Point& p : track_points) {
            Point point(-p.y, p.x, p.z);
            point = transformPoint(point, base_frame_, current_frame_id_);
            relative_track_points.emplace_back(point);
        }

        publishPoint(point_publisher_, relative_avg, relative_track_points);
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

        Point final = computeAveragePoint(non_outliers);

        if (non_outliers.size() == sample_size_) {
            return final;
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
