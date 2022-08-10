/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_FRAME_PUBLISHER__POINT_CLOUD_ALIGNER_H
#define MARCH_FRAME_PUBLISHER__POINT_CLOUD_ALIGNER_H

#include "march_shared_msgs/msg/foot_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <frame_publisher.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FramePublisher;

class PointCloudAligner {
public:
    explicit PointCloudAligner(FramePublisher* n, std::string left_or_right);

    ~PointCloudAligner() = default;

    void alignPointCloud();

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

protected:
    FramePublisher* n_;
    std::string left_or_right_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Subscription<march_shared_msgs::msg::FootPosition>::SharedPtr
        point_subscriber_;

    geometry_msgs::msg::Point last_point_;
    int point_count_;

    double min_check_angle_;
    double max_check_angle_;
    double angle_offset_;
    int avg_sample_size_;
    int num_skip_points_;
    int binary_steps_;

    bool binary_search_;

    double linearSearchOptimalAngle();

    double binarySearchOptimalAngle();

    double computeAverageHeight(double angle);

    void pointCallback(
        const march_shared_msgs::msg::FootPosition::SharedPtr msg);

    void parameterUpdatedLogger(const rclcpp::Parameter& param);
};

#endif // MARCH_FRAME_PUBLISHER__POINT_CLOUD_ALIGNER_H
