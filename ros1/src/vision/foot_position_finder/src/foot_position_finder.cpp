#include "std_msgs/String.h"
#include "utilities/realsense_to_pcl.hpp"
#include "preprocessor.h"
#include "point_finder.h"
#include "foot_position_finder.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <string>
#include <iostream>


FootPositionFinder::FootPositionFinder(ros::NodeHandle* n, bool realsense, char left_or_right)
    : n_(n)
    , realsense_(realsense)
    , left_or_right_(left_or_right)
{

    if (left_or_right_ == 'l')
    {
        point_publisher_ = n_->advertise<std_msgs::String>("/foot_position/left", 1);
        preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>("/camera_left/preprocessed_cloud", 1);
    }
    else
    {
        point_publisher_ = n_->advertise<std_msgs::String>("/foot_position/right", 1);
        preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>("/camera_right/preprocessed_cloud", 1);
    }

    if (realsense)
    {
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);
        processRealSenseDepthFrames();
    }
    else
    {
        if (left_or_right_ == 'l')
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
    ROS_INFO("Processing");
    NormalCloud::Ptr normalcloud(new NormalCloud());
    NormalsPreprocessor preprocessor(pointcloud, normalcloud);
    preprocessor.preprocess();

    publishCloud(preprocessed_pointcloud_publisher_, *pointcloud);

    std::vector<double> search_region = {-0.5, 0.5, 0, 1, -2, 2};
    PointFinder pointFinder(pointcloud, search_region, left_or_right_);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    if (position_queue.size() > 0) {
        ROS_INFO("Found!");
        Point p = position_queue[0];
        computeTemporalAveragePoint(p);
    }
    return true;
}


bool FootPositionFinder::computeTemporalAveragePoint(Point &new_point)
{
    if (found_points_.size() < sample_size_)
        found_points_.push_back(new_point);
    else
    {
        std::rotate(found_points_.begin(), found_points_.begin() + 1, found_points_.end());
        found_points_[sample_size_-1] = new_point;

        double x_avg = 0, y_avg = 0, z_avg = 0;

        for (Point &p : found_points_) {
            x_avg += new_point.x; y_avg += new_point.y; z_avg += new_point.z;
        }

        x_avg /= sample_size_; y_avg /= sample_size_; z_avg /= sample_size_;
        Point avg(x_avg, y_avg, z_avg);

        std::vector<Point> non_outliers;

        for (Point &p : found_points_) {
            if (sqrt(pow(p.x - avg.x, 2) + pow(p.y - avg.y, 2) +  pow(p.z - avg.z, 2)) < 0.05)
                non_outliers.push_back(p);
        }
        if (non_outliers.size() < sample_size_)
            std::cout << non_outliers.size() << std::endl;

        x_avg = 0, y_avg = 0, z_avg = 0;
        for (Point &p : non_outliers) {
            x_avg += p.x; y_avg += p.y; z_avg += p.z;
        }
        x_avg /= sample_size_; y_avg /= sample_size_; z_avg /= sample_size_;
        Point found_point(x_avg, y_avg, z_avg);

        if (non_outliers.size() == sample_size_)
            publishNextPoint(found_point);
    }
    return true;
}

bool FootPositionFinder::publishNextPoint(Point &p)
{
    if (left_or_right_ == 'l')
    {
        ROS_INFO("Left point: (%d, %d, %d)", p.data[0], p.data[1], p.data[2]);
    }
    else if (left_or_right_ == 'r')
    {
        ROS_INFO("Right point: (%d, %d, %d)", p.data[0], p.data[1], p.data[2]);
    }
    return true;
}


void FootPositionFinder::publishCloud(const ros::Publisher& publisher, PointCloud cloud)
{
    cloud.width = 1;
    cloud.height = cloud.points.size();

    sensor_msgs::PointCloud2 msg;

    pcl::toROSMsg(cloud, msg);

    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    publisher.publish(msg);
}

