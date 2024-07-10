#define BOOST_BIND_NO_PLACEHOLDERS
#include "processing/camera_interface.h"
#include "elevation_mapping/elevation_mapping.hpp"
#include "plane_segmentation/plane_segmentation_pipeline.h"
#include "computer_vision_node.h"

namespace march_vision {

ComputerVisionNode::ComputerVisionNode(): Node("computer_vision_node")
{
    RCLCPP_INFO(get_logger(), "ComputerVisionNodeeee is initializing...");
    declareParameters();
    RCLCPP_INFO(get_logger(), "ComputerVisionNode has been created.\n On standby for configuration.");

    configureParameters();
    configurePublishers();
    configureSubscriptions();
    configureCameras();

    // // For Cybathlon specific setup
    // m_exo_mode_sub = create_subscription<march_shared_msgs::msg::ExoMode>(
    //     "/current_mode", 10, std::bind(&ComputerVisionNode::exoModeCallback, this, std::placeholders::_2));
}

void ComputerVisionNode::declareParameters()
{
    declare_parameter("cameras_used", std::string());   
    this->declare_parameter("plane_segmentation", true);
}

void ComputerVisionNode::configureParameters()
{
    this->get_parameter("cameras_used", m_cameras_used);
    this->get_parameter("plane_segmentation", m_plane_segmentation);
    RCLCPP_INFO(get_logger(), "Cameras used: %s", m_cameras_used.c_str());
    RCLCPP_INFO(get_logger(), "Plane segmentation: %s", m_plane_segmentation ? "true" : "false");

}

bool ComputerVisionNode::configureCameras()
{
    RCLCPP_INFO(get_logger(), "Configuring cameras...");
    if (m_cameras_used == "both" || m_cameras_used == "left") {
        m_left_camera_interface = std::make_shared<CameraInterface>(this, std::string("left"));
        if (!m_left_camera_interface->initializeCamera()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize left camera");
            return false;
        }
    }
    if (m_cameras_used == "both" || m_cameras_used == "right") {
        m_right_camera_interface = std::make_shared<CameraInterface>(this, std::string("right"));
        if (!m_right_camera_interface->initializeCamera()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize right camera");
            return false;
        }
    }
    return true;
}

void ComputerVisionNode::configurePublishers()
{
    m_left_synced_pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cv_left_pc", 10);
    m_right_synced_pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cv_right_pc", 10);
}

void ComputerVisionNode::configureSubscriptions()
{
    subscribeToCamera("left", m_cameras_used == "both" || m_cameras_used == "left");
    subscribeToCamera("right", m_cameras_used == "both" || m_cameras_used == "right");

    if (m_cameras_used == "both") {
        
        // Assuming dual camera callback setup
        // m_pointclouds_sync.reset(new message_filters::Synchronizer<m_sync_policy>(m_sync_policy(10), m_left_camera_sub, m_right_camera_sub));
        // m_pointclouds_sync->registerCallback(std::bind(&ComputerVisionNode::dualCameraCallback, this));
    }
}

void ComputerVisionNode::subscribeToCamera(const std::string& camera_side, bool subscribe)
{
    if (!subscribe) return;

    auto topic_name = "cameras_" + camera_side + "/depth/color/points";
    auto singleCameraCallback = std::bind(&ComputerVisionNode::singleCameraCallback, this, std::placeholders::_1);
    auto dualCamerasCallback = std::bind(&ComputerVisionNode::dualCameraCallback, this, std::placeholders::_1, std::placeholders::_2);

    if (camera_side == "left") {
        m_left_camera_sub = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, singleCameraCallback);
    } else if (camera_side == "right") {
        m_right_camera_sub = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 10, singleCameraCallback);
    }

    if (m_cameras_used == "both") {
        // Assuming dual camera callback setup
        // m_pointclouds_sync.reset(new message_filters::Synchronizer<m_sync_policy>(m_sync_policy(10), m_left_camera_sub, m_right_camera_sub));
        // m_pointclouds_sync->registerCallback(dualCamerasCallback);
    }
}

void ComputerVisionNode::singleCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received point cloud from camera");
}

void ComputerVisionNode::dualCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr left_msg,
                                           const sensor_msgs::msg::PointCloud2::SharedPtr right_msg)
{
    RCLCPP_INFO(get_logger(), "Received point clouds from both cameras");
}

void ComputerVisionNode::exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "exO MODE CALBACK");

}

}  // namespace march_vision

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<march_vision::ComputerVisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}