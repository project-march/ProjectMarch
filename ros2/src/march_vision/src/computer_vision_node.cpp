#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "march_vision/processing/camera_interface.hpp"
#include "march_vision/elevation_mapping/elevation_mapping.hpp"
#include "march_vision/plane_segmentation/plane_segmentation.hpp"

ComputerVisionNode::ComputerVisionNode(): LifecycleNode("march_vision")
{
    RCLCPP_INFO(this->get_logger(), "ComputerVisionNode is initializing...");
    declareParameters();
    RCLCPP_INFO(this->get_logger(), "ComputerVisionNode has been created.\nOn standby for configuration.");
}

ComputerVisionNode::~ComputerVisionNode()
{
    RCLCPP_WARN(this->get_logger(), "ComputerVisionNode has been destroyed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_configure(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "Computer Vision is configuring...");

    configureSubscriptions();
    configurePublishers();
    configureParameters();

    // TODO: See if we want tf here or in elevation mapping
    // configureTF2();

    if (!configureCameras()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize cameras");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    if (!configureElevationMapping()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize elevation mapping.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    if (m_plane_segmentation && !configurePlaneSegmentation()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize plane segmentation.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "Computer Vision Node is fully configured.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_activate(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "Computer Vision Node is activating...");

    
    RCLCPP_INFO(this->get_logger(), "Computer Vision Node is active");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "Computer Vision Node is deactivating...");

    RCLCPP_INFO(this->get_logger(), "Computer Vision Node is inactive");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_cleanup(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "Computer Vision Node is cleaning up...");

    RCLCPP_INFO(this->get_logger(), "Computer Vision Node has been cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "Computer Vision Node is shutting down...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ComputerVisionNode::declareParameters()
{
    declare_parameter("cameras_used", std::string());
    declare_parameter("left_camera_serial_number", int());
    declare_parameter("right_camera_serial_number", int());
    // Realsense cameras settings
    declare_parameter("realsense_camera_settings.decimation_filter", bool());
    declare_parameter("realsense_camera_settings.spatial_filter", bool());
    declare_parameter("realsense_camera_settings.temporal_filter", bool());
    declare_parameter("realsense_camera_settings.hole_filling_filter", bool());
    declare_parameter("realsense_camera_settings.threshold_filter", bool());

    declare_parameter("plane_segmentation", bool());
}
void ComputerVisionNode::configureParameters()
{
    // TODO: Check what default should be 
    m_exo_mode = "Unknown"

    get_parameter("cameras_used", m_cameras_used);
    get_parameter("left_camera_serial_number", m_left_camera_serial_number);
    get_parameter("right_camera_serial_number", m_right_camera_serial_number);
    // Realsense cameras settings
    get_parameter("realsense_camera_settings.decimation_filter", m_realsense_camera_settings.decimation_filter);
    get_parameter("realsense_camera_settings.spatial_filter", m_realsense_camera_settings.spatial_filter);
    get_parameter("realsense_camera_settings.temporal_filter", m_realsense_camera_settings.temporal_filter);
    get_parameter("realsense_camera_settings.hole_filling_filter", m_realsense_camera_settings.hole_filling_filter);
    get_parameter("realsense_camera_settings.threshold_filter", m_realsense_camera_settings.threshold_filter);

    get_parameter("plane_segmentation", m_plane_segmentation);
}
bool ComputerVision::configureCameras()
{
    // Initialize the camera interfaces
    m_left_camera_interface = CameraInterface(this, "left");
    m_right_camera_interface = CameraInterface(this, "right");

    // Initialize the camera interfaces
    if (!m_left_camera_interface.initializeCamera(m_left_camera_serial_number, m_realsense_camera_settings)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize left camera");
        return false;
    }

    if (!m_right_camera_interface.initializeCamera(m_right_camera_serial_number, m_realsense_camera_settings)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize right camera");
        return false;
    }

    return true;
}

bool ComputerVisionNode::configureElevationMapping()
{
    transition_msg.transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    transition_msg.transition.label = "Configuring";
    m_elevation_mapping_state_pub->publish(transition_msg);
}

bool ComputerVisionNode::configurePlaneSegmentation()
{
    transition_msg.transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    transition_msg.transition.label = "Configuring";
    m_plane_segmentation_state_pub->publish(transition_msg);
}

void ComputerVisionNode::configurePublishers()
{
    rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_elevation_mapping_state_pub;
    rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_plane_segmentation_state_pub;
    m_elevation_mapping_state_pub = this->create_publisher<lifecycle_msgs::msg::Transition>("/elevation_mapping/transition_event", 10);
    m_plane_segmentation_state_pub = this->create_publisher<lifecycle_msgs::msg::Transition>("/plane_segmentation/transition_event", 10);
    lifecycle_msgs::msg::Transition transition_msg;
}

void ComputerVisionNode::configureSubscriptions()
{
    m_exo_mode_sub = this->create_subscription<std_msgs::msg::ExoMode>("current_mode", 
                                               10, std::bind(&ComputerVisionNode::exoModeCallback, this, std::placeholders::_1));
    m_left_camera_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cameras_left/depth/color/points", 100);
    m_right_camera_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cameras_right/depth/color/points", 100);

    // Syncing the left and right camera topics
    m_sync = std::make_shared<message_filters::Synchronizer<m_sync_policy>>(m_sync_policy(100), *m_left_camera_sub, *m_right_camera_sub);
    m_sync->registerCallback(std::bind(&InputSourceManagerNode::dualCameraCallback, this, std::placeholders::_1, std::placeholders::_2));

}

void ComputerVisionNode::dualCameraCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr left_msg, 
    const sensor_msgs::msg::PointCloud2::SharedPtr right_msg)
{
    m_left_camera_interface.processPointCloud(left_msg);
    m_right_camera_interface.processPointCloud(right_msg);
}

void ComputerVisionNode::exoModeCallback(const std_msgs::msg::ExoMode::SharedPtr msg)
{
    // TODO: Implement switch case / state machine for different modes
    m_exo_mode = msg->mode.to_string();
}