#ifndef COMPUTER_VISION_NODE_HPP_
#define COMPUTER_VISION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_vision/processing/camera_interface.h"

namespace march_vision {
    
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> m_sync_policy;

class ComputerVisionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    ComputerVisionNode();
    ~ComputerVisionNode();

protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    void declareParameters();
    void configureParameters();
    bool configureCameras();
    void configurePublishers();
    void configureSubscriptions();
    void singleCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void dualCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr left_msg, const sensor_msgs::msg::PointCloud2::SharedPtr right_msg);
    void exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);

    CameraInterface m_left_camera_interface;
    CameraInterface m_right_camera_interface;
    std::string m_cameras_used;
    bool m_plane_segmentation;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> m_left_camera_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> m_right_camera_sub;
    std::shared_ptr<message_filters::Synchronizer<m_sync_policy>> m_pointclouds_sync;
    std::shared_ptr<rclcpp::Subscription<march_shared_msgs::msg::ExoMode>> m_exo_mode_sub;
    rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_elevation_mapping_state_pub;
    rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_plane_segmentation_state_pub;
};

}  // namespace march_vision

#endif  // COMPUTER_VISION_NODE_HPP_