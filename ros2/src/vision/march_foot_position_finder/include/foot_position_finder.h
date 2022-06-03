/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_FOOT_POSITION_FINDER_H
#define MARCH_FOOT_POSITION_FINDER_H

#include "march_shared_msgs/msg/current_state.hpp"
#include "march_shared_msgs/msg/foot_position.hpp"
#include "point_finder.h"
#include "preprocessor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <librealsense2/rs.hpp>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <vector>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FootPositionFinder {
public:
    explicit FootPositionFinder(
        rclcpp::Node* n, const std::string& left_or_right);

    void readParameters(const std::vector<rclcpp::Parameter>& parameters);

    ~FootPositionFinder() = default;

protected:
    void currentStateCallback(
        const march_shared_msgs::msg::CurrentState::SharedPtr msg);

    void resetInitialPosition(bool stop_timer);

    void chosenOtherPointCallback(
        const march_shared_msgs::msg::FootPosition::SharedPtr msg);

    void processRealSenseDepthFrames();

    void processSimulatedDepthFrames(
        const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud);

    void processPointCloud(const PointCloud::Ptr& pointcloud);

    Point computeTemporalAveragePoint(const Point& new_point);

    Point transformPoint(Point point, const std::string& frame_from,
        const std::string& frame_to);

    rclcpp::Node* n_;
    std::unique_ptr<Preprocessor> preprocessor_ { nullptr };
    std::unique_ptr<PointFinder> point_finder_ { nullptr };

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ { nullptr };
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ { nullptr };

    rclcpp::Publisher<march_shared_msgs::msg::FootPosition>::SharedPtr
        point_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointcloud_subscriber_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        preprocessed_pointcloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        point_marker_publisher_;
    rclcpp::Subscription<march_shared_msgs::msg::FootPosition>::SharedPtr
        other_chosen_point_subscriber_;
    rclcpp::Subscription<march_shared_msgs::msg::CurrentState>::SharedPtr
        current_state_subscriber_;

    rclcpp::TimerBase::SharedPtr realsense_timer_;
    rclcpp::TimerBase::SharedPtr initial_position_reset_timer_;

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
};

#endif // MARCH_FOOT_POSITION_FINDER_H
