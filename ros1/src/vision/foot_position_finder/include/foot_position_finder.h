#ifndef MARCH_FOOT_POSITION_FINDER_H
#define MARCH_FOOT_POSITION_FINDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <librealsense2/rs.hpp>


using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FootPositionFinder {
public:

    explicit FootPositionFinder(ros::NodeHandle* n, bool realsense, char left_or_right);

    ~FootPositionFinder() = default;

protected:

    bool processRealSenseDepthFrames();

    void processSimulatedDepthFrames(const sensor_msgs::PointCloud2 input_cloud);

    bool processPointCloud(PointCloud::Ptr pointcloud);

    bool computeTemporalAveragePoint(Point &new_point);

    bool publishNextPoint(Point &p);

    void publishCloud(const ros::Publisher& publisher, PointCloud cloud);


    rs2::pipeline pipe;
    rs2::config cfg;

    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    bool realsense_;

    ros::NodeHandle* n_;

    ros::Publisher point_publisher_;
    ros::Subscriber pointcloud_subscriber_;

    std::string TOPIC_CAMERA_FRONT_LEFT = "/camera_front_left/depth/color/points";
    std::string TOPIC_CAMERA_FRONT_RIGHT = "/camera_front_right/depth/color/points";
    std::string TOPIC_TEST_CLOUDS = "/test_clouds";

    std::vector<Point> found_points_;
    int sample_size_ = 5;
    char left_or_right_;

    ros::Publisher preprocessed_pointcloud_publisher_;

};

#endif // MARCH_FOOT_POSITION_FINDER_H