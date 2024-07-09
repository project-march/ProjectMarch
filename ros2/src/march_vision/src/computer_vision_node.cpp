// #include "march_vision/processing/camera_interface.h"
// #include "march_vision/elevation_mapping/elevation_mapping.hpp"
// #include "march_vision/plane_segmentation/plane_segmentation.hpp"

// ComputerVisionNode::ComputerVisionNode(): LifecycleNode("march_vision")
// {
//     RCLCPP_INFO(this->get_logger(), "ComputerVisionNode is initializing...");
//     declareParameters();
//     RCLCPP_INFO(this->get_logger(), "ComputerVisionNode has been created.\n On standby for configuration.");
// }

// ComputerVisionNode::~ComputerVisionNode()
// {
//     RCLCPP_WARN(this->get_logger(), "ComputerVisionNode has been destroyed.");
// }

// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_configure(const rclcpp_lifecycle::State& state)
// {
//     (void) state;
//     RCLCPP_INFO(this->get_logger(), "Computer Vision is configuring...");

//     configureSubscriptions();
//     configurePublishers();
//     configureParameters();

//     // TODO: See if we want tf here or in elevation mapping
//     // configureTF2();

//     if (!configureCameras()) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to initialize cameras");
//         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
//     }
//     lifecycle_msgs::msg::Transition transition_msg = lifecycle_msgs::msg::Transition(1);
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }

//     // TODO: Implement checks here if el. mapping and pl. seg. are successfully configured

//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node is fully configured.");
//     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
// }

// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_activate(const rclcpp_lifecycle::State& state)
// {
//     (void) state;
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node is activating...");
//     lifecycle_msgs::msg::Transition transition_msg = lifecycle_msgs::msg::Transition(3);
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     // TODO: Activate callbacks timers maybe?
//     if (m_cameras_used == "both") {
//         m_left_camera_interface.run();
//         m_right_camera_interface.run();
//     } else if (m_cameras_used == "left") {
//         m_left_camera_interface.run();
//     } else if (m_cameras_used == "right") {
//         m_right_camera_interface.run();
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "No cameras are being used. No processes to be activated.");
//         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
//     }
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node is active");
//     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
// }

// // TODO: Cancel the walltimers instead maybe?
// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_deactivate(const rclcpp_lifecycle::State& state)
// {
//     (void) state;
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node is deactivating...");
//     lifecycle_msgs::msg::Transition transition_msg = lifecycle_msgs::msg::Transition(4);
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     if (m_cameras_used == "both") {
//         m_left_camera_interface.stop();
//         m_right_camera_interface.stop();
//     } else if (m_cameras_used == "left") {
//         m_left_camera_interface.stop();
//     } else if (m_cameras_used == "right") {
//         m_right_camera_interface.stop();
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "No cameras are being used. No processes to be stopped.");
//         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
//     }
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node is inactive");
//     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
// }

// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_cleanup(const rclcpp_lifecycle::State& state)
// {
//     (void) state;
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node is cleaning up...");
//     lifecycle_msgs::msg::Transition transition_msg = lifecycle_msgs::msg::Transition(2);
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     // TODO: How to clean up the cameras?
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node has been cleaned up");
//     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
// }

// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_shutdown(const rclcpp_lifecycle::State& state)
// {
//     (void) state;
//     lifecycle_msgs::msg::Transition transition_msg = lifecycle_msgs::msg::Transition(7);
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     if (m_cameras_used == "both") {
//         m_left_camera_interface.shutdown();
//         m_right_camera_interface.shutdown();
//     } else if (m_cameras_used == "left") {
//         m_left_camera_interface.shutdown();
//     } else if (m_cameras_used == "right") {
//         m_right_camera_interface.shutdown();
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "No cameras are being used. No processes to shut down.");
//         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
//     }
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node has shut down.");
//     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
// }

// void ComputerVisionNode::declareParameters()
// {
//     declare_parameter("m_cameras_used", std::string());
//     declare_parameter("m_plane_segmentation", true);
//     declare_parameter("m_latest_left_pc", PointCloud2::SharedPtr());
//     declare_parameter("m_latest_right_pc", PointCloud2::SharedPtr());
//     std::mutex pc_mutex;
// }

// void ComputerVisionNode::configureParameters()
// {
//     // m_exo_mode = "Idle";
//     m_cameras_used = this->get_parameter("cameras_used").as_string();
//     m_plane_segmentation = this->get_parameter("plane_segmentation").as_bool();
// }

// bool ComputerVisionNode::configureCameras()
// {
//     if (m_cameras_used == "both" || m_cameras_used == "left") {
//         m_left_camera_interface = CameraInterface(this, "left");
//         if (!m_left_camera_interface.initializeCamera()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to initialize left camera");
//             return false;
//         }
//     }
//     if (m_cameras_used == "both" || m_cameras_used == "right") {
//         m_right_camera_interface = CameraInterface(this, "right");
//         if (!m_right_camera_interface.initializeCamera()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to initialize right camera");
//             return false;
//         }
//     }
//     return true;
// }

// void ComputerVisionNode::configurePublishers()
// {
//     // TODO: Check if this is the correct way to do this in C++
//     rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_elevation_mapping_state_pub;
//     rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_plane_segmentation_state_pub;
//     m_elevation_mapping_state_pub = this->create_publisher<lifecycle_msgs::msg::Transition>("/elevation_mapping/transition_event", 10);
//     m_plane_segmentation_state_pub = this->create_publisher<lifecycle_msgs::msg::Transition>("/plane_segmentation/transition_event", 10);
// }

// void ComputerVisionNode::configureSubscriptions()
// {
//     // Simplified and refactored subscription logic
//     subscribeToCamera("left", m_cameras_used == "both" || m_cameras_used == "left");
//     subscribeToCamera("right", m_cameras_used == "both" || m_cameras_used == "right");

//     if (m_cameras_used == "both") {
//         m_pointclouds_sync.reset(new message_filters::Synchronizer<m_sync_policy>(m_sync_policy(10), m_left_camera_sub, m_right_camera_sub));
//         m_pointclouds_sync->registerCallback(std::bind(&ComputerVisionNode::dualCameraCallback, this));
//     }
//     m_exo_mode_sub = std::make_shared<rclcpp::Subscription<march_shared_msgs::msg::ExoMode>>(this, "/current_mode", 10);

// }

// void ComputerVisionNode::subscribeToCamera(const std::string& camera_side, bool subscribe)
// {
//     if (!subscribe) return;

//     auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "cameras_" + camera_side + "/depth/color/points", 10);
//     if (camera_side == "left") {
//         m_left_camera_sub = sub;
//     } else if (camera_side == "right") {
//         m_right_camera_sub = sub;
//     }
//     if (m_cameras_used != "both") {
//         sub->registerCallback(std::bind(&ComputerVisionNode::singleCameraCallback, this, std::placeholders::_1));
//     }
// }

// void ComputerVisionNode::singleCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//     // TODO: Check if these are the correct header frame ids
//     // TODO: Check if this is the correct way to process point clouds
//     if (msg->header.frame_id == "left_camera") {
//         m_cv_left_pc->publish(left_msg);
//     } else if (msg->header.frame_id == "right_camera") {
//         m_cv_right_pc->publish(right_msg);
//     }
// }

// void ComputerVisionNode::dualCameraCallback(
//     const sensor_msgs::msg::PointCloud2::SharedPtr left_msg, 
//     const sensor_msgs::msg::PointCloud2::SharedPtr right_msg)
// {
//     // m_left_camera_interface.processPointCloud(left_msg);
//     // m_right_camera_interface.processPointCloud(right_msg);
//     std::lock_guard<std::mutex> lock(pc_mutex);
//     // pointCloudRegistration(m_pc_registration_method, left_msg, right_msg);
//     m_cv_left_pc->publish(left_msg);
//     m_cv_right_pc->publish(right_msg);
// }

// // Cybathlon specific setup
// void ComputerVisionNode::exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg)
// {
//     // TODO: Implement switch case / state machine for different modes
//     m_exo_mode = msg->mode.to_string();
// }

int main(int argc, char* argv[])
{
    // rclcpp::init(argc, argv);
    // // TODO: Multi or single threaded executor?
    // rclcpp::executors::MultiThreadedExecutor executor;
    // auto node = std::make_shared<ComputerVisionNode>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}