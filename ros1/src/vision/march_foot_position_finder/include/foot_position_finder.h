/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_FOOT_POSITION_FINDER_H
#define MARCH_FOOT_POSITION_FINDER_H

#include "march_shared_msgs/FootPosition.h"
#include <chrono>
#include <cmath>
#include <librealsense2/rs.hpp>
#include <march_foot_position_finder/parametersConfig.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_listener.h>
#include <thread>
#include <vector>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FootPositionFinder {
public:
    explicit FootPositionFinder(
        ros::NodeHandle* n, const std::string& left_or_right);

    void readParameters(
        march_foot_position_finder::parametersConfig& config, uint32_t level);

    ~FootPositionFinder() = default;

protected:
    void chosenOtherPointCallback(const march_shared_msgs::FootPosition msg);

    void processRealSenseDepthFrames(const ros::TimerEvent&);

    void processSimulatedDepthFrames(
        const sensor_msgs::PointCloud2 input_cloud);

    void processPointCloud(const PointCloud::Ptr& pointcloud);

    Point computeTemporalAveragePoint(const Point& new_point);

    Point transformPoint(Point point, const std::string& frame_from,
        const std::string& frame_to);

    ros::NodeHandle* n_;

    ros::Publisher point_publisher_;
    ros::Subscriber pointcloud_subscriber_;

    ros::Publisher preprocessed_pointcloud_publisher_;
    ros::Publisher point_marker_publisher_;
    ros::Subscriber current_chosen_point_subscriber_;
    ros::Subscriber other_chosen_point_subscriber_;

    ros::Publisher height_map_publisher_;

    ros::Timer realsense_timer_;
    ros::Timer height_reset_timer_;

    clock_t last_frame_time_;
    int frame_wait_counter_;
    float frame_timeout_;

    rs2::pipeline pipe_;
    rs2::config config_;
    std::string serial_number_;

    rs2::decimation_filter dec_filter_;
    rs2::spatial_filter spat_filter_;
    rs2::temporal_filter temp_filter_;

    std::string topic_camera_front_;
    std::string topic_current_chosen_point_;
    std::string topic_other_chosen_point_;

    std::string left_or_right_;
    std::string other_side_;

    std::string other_frame_id_;
    std::string current_frame_id_;

    bool running_;
    bool realsense_simulation_;

    double foot_gap_;
    double step_distance_;
    double outlier_distance_;
    double height_zero_threshold_;
    float last_height_;
    int switch_factor_;
    int sample_size_;
    int refresh_last_height_;

    std::vector<Point> found_points_;

    Point ORIGIN;
    Point last_chosen_point_;
    Point start_point_;
    Point desired_point_;
    Point previous_start_point_;
    Point last_displacement_;
    Point new_displacement_;
    Point found_covid_point_;

    tf::TransformListener listener;
};

#endif // MARCH_FOOT_POSITION_FINDER_H
