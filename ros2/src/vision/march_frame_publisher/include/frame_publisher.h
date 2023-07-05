/**
 * @author Jelmer de Wolde, Tuhin Das - MARCH 7
 */

#ifndef MARCH_FRAME_PUBLISHER__FRAME_PUBLISHER_H
#define MARCH_FRAME_PUBLISHER__FRAME_PUBLISHER_H

#include "march_shared_msgs/msg/foot_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <point_cloud_aligner.h>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

using namespace std::chrono_literals;

class PointCloudAligner;

class FramePublisher : public rclcpp::Node {
public:
    explicit FramePublisher();

    void setLeftCameraRotation(double angle);

    void setRightCameraRotation(double angle);

private:
    std::shared_ptr<rclcpp::Node> client_node_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_ { nullptr };
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<PointCloudAligner> left_aligner_ { nullptr };
    std::unique_ptr<PointCloudAligner> right_aligner_ { nullptr };

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr align_service_;
    std::shared_ptr<rclcpp::SyncParametersClient> vision_parameter_client_;

    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_threshold_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_threshold_client_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string robot_properties_path
        = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/properties/properties_march8.yaml";
    YAML::Node robot_properties = YAML::LoadFile(robot_properties_path);
    const double TRANS_X = -robot_properties["dimensions"]["foot"]["height_forward"].as<double>();
    const double TRANS_Z = -0.025;

    geometry_msgs::msg::TransformStamped trans_left_;
    geometry_msgs::msg::TransformStamped trans_right_;
    rclcpp::Time last_published_left_;
    rclcpp::Time last_published_right_;
    bool start_time_initialized_ = false;

    double rotation_camera_left_ = 0.0;
    double rotation_camera_right_ = 0.0;

    rclcpp::CallbackGroup::SharedPtr timer_group_;

    geometry_msgs::msg::TransformStamped tr;
    rclcpp::Time last_published_camera_left_ = tr.header.stamp;
    rclcpp::Time last_published_camera_right_ = tr.header.stamp;

    rcl_interfaces::msg::ParameterDescriptor descriptor_;
    rcl_interfaces::msg::FloatingPointRange range_;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    void parameterUpdatedLogger(const rclcpp::Parameter& param);

    void publishCameraFrame(const std::string& left_or_right);

    void publishToeFrames();

    void alignCamerasCallback();

    void alignCameras(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    double getHeightZeroThreshold();

    void setHeightZeroThreshold(double threshold);
};

#endif // MARCH_FRAME_PUBLISHER__FRAME_PUBLISHER_H
