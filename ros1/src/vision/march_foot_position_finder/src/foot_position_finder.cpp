#include "foot_position_finder.h"
#include "point_finder.h"
#include "preprocessor.h"
#include "std_msgs/String.h"
#include "utilities/math_utilities.hpp"
#include "utilities/publish_utilities.hpp"
#include "utilities/realsense_to_pcl.hpp"
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
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
FootPositionFinder::FootPositionFinder(ros::NodeHandle* n, bool realsense,
    const std::string& left_or_right) // NOLINT
    : n_(n)
    , realsense_(realsense)
    , left_or_right_(left_or_right)
{

    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
    topic_camera_front
        = "/camera_front_" + left_or_right + "/depth/color/points";

    point_publisher_ = n_->advertise<geometry_msgs::Point>(
        "/foot_position/" + left_or_right_, /*queue_size=*/1);
    preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>(
        "/camera_" + left_or_right_ + "/preprocessed_cloud", /*queue_size=*/1);
    point_marker_publisher_ = n_->advertise<visualization_msgs::Marker>(
        "/camera_" + left_or_right_ + "/found_points", /*queue_size=*/1);

    if (left_or_right_ == "left") {
        reference_frame_id = "foot_right";
    } else if (left_or_right_ == "right") {
        reference_frame_id = "foot_left";
    }

    if (realsense) {
        cfg.enable_stream(RS2_STREAM_DEPTH, /*width=*/640, /*height=*/480,
            RS2_FORMAT_Z16, /*framerate=*/30);
        pipe.start(cfg);
        processRealSenseDepthFrames();
    } else {
        pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(
            topic_camera_front, /*queue_size=*/1,
            &FootPositionFinder::processSimulatedDepthFrames, this);
    }
}

/**
 * Listen for realsense frames from a camera, apply filters to them and process
 * the eventual pointcloud.
 */
void FootPositionFinder::processRealSenseDepthFrames()
{
    while (ros::ok()) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();

        depth = dec_filter.process(depth);
        depth = spat_filter.process(depth);
        depth = temp_filter.process(depth);

        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);

        PointCloud::Ptr pointcloud = points_to_pcl(points);
        processPointCloud(pointcloud);
        ros::spinOnce();
    }
}

/**
 * Callback function for when a simulated realsense depth frame arrives.
 */
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
        if (left_or_right_ == "left") {
            point = Point(/*_x=*/-0.5, /*_y=*/-0.25, /*_z=*/0);
        } else if (left_or_right_ == "right") {
            point = Point(/*_x=*/-0.5, /*_y=*/0.25, /*_z=*/0);
        }

        // Calculate point location relative to positionary leg
        transformPoint(point, reference_frame_id, "world");
    }

    Preprocessor preprocessor(pointcloud, normalcloud);
    preprocessor.preprocess();

    publishCloud(preprocessed_pointcloud_publisher_, *pointcloud);

    Point position(point.y, -point.x, point.z); // Rotate 90 degrees clockwise
    PointFinder pointFinder(pointcloud, left_or_right_, position);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    if (position_queue.size() > 0) {
        Point p = position_queue[0];
        computeTemporalAveragePoint(position_queue[0]);
    }
}

/**
 * Computes a temporal average of the last X points and removes any outliers,
 * then publishes the average of the points without the outliers.
 */
void FootPositionFinder::computeTemporalAveragePoint(const Point& new_point)
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
            if (pcl::squaredEuclideanDistance(p, avg) < 0.05) {
                non_outliers.push_back(p);
            }
        }

        Point final = computeAveragePoint(non_outliers);

        if (non_outliers.size() == sample_size_) {
            // Publish for visualization
            publishMarkerPoint(point_marker_publisher_, final);

            // Publish for gait computation
            final = Point(
                -final.y, final.x, final.z); // Rotate 90 counter clockwise
            transformPoint(final, "world", reference_frame_id);
            publishPoint(point_publisher_, final);
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
void FootPositionFinder::transformPoint(
    Point& point, std::string frame_from, std::string frame_to)
{
    PointCloud::Ptr desired_point = boost::make_shared<PointCloud>();
    desired_point->push_back(point);

    geometry_msgs::TransformStamped transform_stamped;

    try {
        if (tfBuffer->canTransform(
                frame_to, frame_from, ros::Time(), ros::Duration(/*t=*/1.0))) {
            transform_stamped = tfBuffer->lookupTransform(
                frame_to, frame_from, ros::Time(/*t=*/0));
        }
    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM(
            "Something went wrong when transforming the pointcloud: "
            << ex.what());
    }

    pcl_ros::transformPointCloud(
        *desired_point, *desired_point, transform_stamped.transform);

    point = desired_point->points[0];
}
