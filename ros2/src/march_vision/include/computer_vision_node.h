#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
// #include "processing/camera_inerface.h"  // Replace with actual header for CameraInterface

// #include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "processing/camera_interface.h"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include <mutex>
#include <string>
#include <memory>

namespace march_vision {

class ComputerVisionNode : public rclcpp::Node {
public:
    explicit ComputerVisionNode();

private:
    void declareParameters();
    void configureParameters();
    bool configureCameras();
    void configurePublishers();
    void configureSubscriptions();
    void subscribeToCamera(const std::string& camera_side, bool subscribe);
    void singleCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void dualCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr left_msg,
                            const sensor_msgs::msg::PointCloud2::SharedPtr right_msg);
    void exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_left_synced_pc_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_right_synced_pc_pub;
    // rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_left_camera_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_right_camera_sub;
    std::shared_ptr<CameraInterface> m_left_camera_interface;
    std::shared_ptr<CameraInterface> m_right_camera_interface;
    std::mutex m_mutex;
    std::string m_cameras_used;
    bool m_plane_segmentation;
    ExoMode m_exo_mode;
};

}  // namespace march_vision
