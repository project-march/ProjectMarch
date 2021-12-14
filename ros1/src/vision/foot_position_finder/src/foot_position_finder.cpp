#include "std_msgs/String.h"
#include "utilities/realsense_to_pcl.hpp"
#include "utilities/publish_utilities.hpp"
#include "preprocessor.h"
#include "point_finder.h"
#include "foot_position_finder.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <string>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>


FootPositionFinder::FootPositionFinder(ros::NodeHandle* n, bool realsense, std::string left_or_right)
    : n_(n)
    , realsense_(realsense)
    , left_or_right_(left_or_right)
{

    tfBuffer = std::make_unique<tf2_ros::Buffer>();
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);

    point_publisher_ = n_->advertise<geometry_msgs::Point>("/foot_position/" + left_or_right_, 1);
    preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>("/camera_" + left_or_right_ + "/preprocessed_cloud", 1);
    found_points_publisher_ = n_->advertise<visualization_msgs::Marker>("/camera_" + left_or_right_ + "/found_points", 1);

    if (realsense)
    {
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);
        processRealSenseDepthFrames();
    }
    else
    {
        if (left_or_right_ == "left")
            pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(TOPIC_CAMERA_FRONT_LEFT, 1, &FootPositionFinder::processSimulatedDepthFrames, this);
        else
            pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(TOPIC_CAMERA_FRONT_RIGHT, 1, &FootPositionFinder::processSimulatedDepthFrames, this);
    }
}


bool FootPositionFinder::processRealSenseDepthFrames()
{
    while (ros::ok())
    {
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
        return true;
    }
    return true;
}


void FootPositionFinder::processSimulatedDepthFrames(const sensor_msgs::PointCloud2 input_cloud)
{    
    PointCloud converted_cloud;
    pcl::fromROSMsg(input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud = boost::make_shared<PointCloud>(converted_cloud);
    processPointCloud(pointcloud);
}


bool FootPositionFinder::processPointCloud(PointCloud::Ptr pointcloud)
{
    
    auto pointcloud_frame_id = pointcloud->header.frame_id.c_str();

    NormalCloud::Ptr normalcloud(new NormalCloud());
    PointCloud::Ptr desired_point = boost::make_shared<PointCloud>();

    if (!realsense_)
    {
        std::string frame_id;
        if (left_or_right_ == "left")
        {
            desired_point->push_back(Point(-0.5, -0.25, 0));
            frame_id = "foot_right";
        }
        else if (left_or_right_ == "right")
        {
            desired_point->push_back(Point(-0.5, 0.25, 0));
            frame_id = "foot_left";
        }
            
        geometry_msgs::TransformStamped transform_stamped;

        try {
            if (tfBuffer->canTransform("world", frame_id, ros::Time(), ros::Duration(1.0)))
                transform_stamped = tfBuffer->lookupTransform("world", frame_id, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_STREAM("Something went wrong when transforming the pointcloud: "<< ex.what());
            return false;
        }

        pcl_ros::transformPointCloud(*desired_point, *desired_point, transform_stamped.transform);
    } 

    Preprocessor preprocessor(pointcloud, normalcloud);
    preprocessor.preprocess();

    publishCloud(preprocessed_pointcloud_publisher_, *pointcloud);

    Point point = desired_point->points[0];
    Point position(point.y, -point.x, point.z);  // Rotate 90 degrees clockwise
    PointFinder pointFinder(pointcloud, left_or_right_, position);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    if (position_queue.size() > 0) {
        publishMarkerPoint(found_points_publisher_, position_queue[0]);
        computeTemporalAveragePoint(position_queue[0]);
    }
    return true;
}

Point FootPositionFinder::computeAveragePoint(const std::vector<Point> &points)
{
    Point avg(0, 0, 0);

    for (const Point &p : points)
    {
        avg.x += p.x;
        avg.y += p.y;
        avg.z += p.z;
    }

    avg.x /= points.size();
    avg.x /= points.size();
    avg.x /= points.size();

    return avg;
}


bool FootPositionFinder::computeTemporalAveragePoint(const Point &new_point)
{
    if (found_points_.size() < sample_size_)
        found_points_.push_back(new_point);
    else
    {
        std::rotate(found_points_.begin(), found_points_.begin() + 1, found_points_.end());
        found_points_[sample_size_-1] = new_point;
        Point avg = computeAveragePoint(found_points_);

        std::vector<Point> non_outliers;
        for (Point &p : found_points_)
            if (pcl::squaredEuclideanDistance(p, avg) < 0.05)
                non_outliers.push_back(p);

        Point found_point = computeAveragePoint(non_outliers);
        if (non_outliers.size() == sample_size_)
            publishNextPoint(found_point);
    }
    return true;
}


bool FootPositionFinder::publishNextPoint(Point &p)
{
    publishMarkerPoint(found_points_publisher_, p);

    geometry_msgs::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point_publisher_.publish(point);
    return true;
}
