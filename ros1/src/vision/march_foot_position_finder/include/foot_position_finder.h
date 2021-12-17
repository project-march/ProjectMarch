#ifndef MARCH_FOOT_POSITION_FINDER_H
#define MARCH_FOOT_POSITION_FINDER_H

#include <cmath>
#include <librealsense2/rs.hpp>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FootPositionFinder {
public:
    explicit FootPositionFinder(
        ros::NodeHandle* n, bool realsense, const std::string& left_or_right);

    ~FootPositionFinder() = default;

protected:
    void processRealSenseDepthFrames();

    void processSimulatedDepthFrames(
        const sensor_msgs::PointCloud2 input_cloud);

    void processPointCloud(const PointCloud::Ptr& pointcloud);

    void computeTemporalAveragePoint(const Point& new_point);

    void publishNextPoint(Point& p);

    rs2::pipeline pipe;
    rs2::config cfg;

    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    ros::NodeHandle* n_;

    ros::Publisher point_publisher_;
    ros::Subscriber pointcloud_subscriber_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;

    std::string topic_camera_front;

    std::vector<Point> found_points_;
    int sample_size_ = 3;
    std::string left_or_right_;
    bool realsense_;

    ros::Publisher preprocessed_pointcloud_publisher_;
    ros::Publisher point_marker_publisher_;
};

#endif // MARCH_FOOT_POSITION_FINDER_H