#ifndef MARCH_FOOT_POSITION_FINDER_H
#define MARCH_FOOT_POSITION_FINDER_H

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>
#include <cmath>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <pcl_ros/transforms.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FootPositionFinder {
public:

    explicit FootPositionFinder(ros::NodeHandle* n, bool realsense, std::string left_or_right);

    ~FootPositionFinder() = default;

protected:

    bool processRealSenseDepthFrames();

    void processSimulatedDepthFrames(const sensor_msgs::PointCloud2 input_cloud);

    bool processPointCloud(PointCloud::Ptr pointcloud);

    bool computeTemporalAveragePoint(const Point &new_point);

    bool publishNextPoint(Point &p);

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

    std::string TOPIC_CAMERA_FRONT_LEFT = "/camera_front_left/depth/color/points";
    std::string TOPIC_CAMERA_FRONT_RIGHT = "/camera_front_right/depth/color/points";
    std::string TOPIC_TEST_CLOUDS = "/test_clouds";

    std::vector<Point> found_points_;
    int sample_size_ = 3;
    std::string left_or_right_;
    bool realsense_;

    ros::Publisher preprocessed_pointcloud_publisher_;
    ros::Publisher found_points_publisher_;

};

#endif // MARCH_FOOT_POSITION_FINDER_H