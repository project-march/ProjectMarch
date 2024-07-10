// #include "processing/camera_interface.h"
// #include "elevation_mapping/elevation_mapping.hpp"
// #include "plane_segmentation/plane_segmentation_pipeline.h"
// #include "computer_vision_node.h"

// namespace march_vision {

// ComputerVisionNode::ComputerVisionNode(): rclcpp_lifecycle::LifecycleNode("march_vision")
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
//     lifecycle_msgs::msg::Transition transition_msg;
//     transition_msg.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
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
//     lifecycle_msgs::msg::Transition transition_msg;
//     transition_msg.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     // TODO: Activate callbacks timers maybe?
//     if (m_cameras_used == "both") {
//         m_left_camera_interface->run();
//         m_right_camera_interface->run();
//     } else if (m_cameras_used == "left") {
//         m_left_camera_interface->run();
//     } else if (m_cameras_used == "right") {
//         m_right_camera_interface->run();
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
//     lifecycle_msgs::msg::Transition transition_msg;
//     transition_msg.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     if (m_cameras_used == "both") {
//         m_left_camera_interface->stop();
//         m_right_camera_interface->stop();
//     } else if (m_cameras_used == "left") {
//         m_left_camera_interface->stop();
//     } else if (m_cameras_used == "right") {
//         m_right_camera_interface->stop();
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
//     lifecycle_msgs::msg::Transition transition_msg;
//     transition_msg.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     // TODO: How to clean up the cameras?
//     RCLCPP_INFO(this->get_logger(), "Computer Vision Node has been cleaned up");
//     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
// }

// rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ComputerVisionNode::on_shutdown(const rclcpp_lifecycle::State& state)
// {
//     (void) state;
//     lifecycle_msgs::msg::Transition transition_msg;
//     transition_msg.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
//     m_elevation_mapping_state_pub->publish(transition_msg);
//     if (m_plane_segmentation) { m_plane_segmentation_state_pub->publish(transition_msg); }
//     if (m_cameras_used == "both") {
//         m_left_camera_interface->shutdown();
//         m_right_camera_interface->shutdown();
//     } else if (m_cameras_used == "left") {
//         m_left_camera_interface->shutdown();
//     } else if (m_cameras_used == "right") {
//         m_right_camera_interface->shutdown();
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
//     std::mutex m_mutex;
// }

// void ComputerVisionNode::configureParameters()
// {
//     // m_exo_mode = "Idle";
//     this->get_parameter("cameras_used", m_cameras_used);
//     this->get_parameter("plane_segmentation", m_plane_segmentation);
// }

// bool ComputerVisionNode::configureCameras()
// {
//     if (m_cameras_used == "both" || m_cameras_used == "left") {
//         m_left_camera_interface = std::make_shared<CameraInterface>(this, "left");
//         if (!m_left_camera_interface->initializeCamera()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to initialize left camera");
//             return false;
//         }
//     }
//     if (m_cameras_used == "both" || m_cameras_used == "right") {
//         m_right_camera_interface = std::make_shared<CameraInterface>(this, "right");
//         if (!m_right_camera_interface->initializeCamera()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to initialize right camera");
//             return false;
//         }
//     }
// }

// void ComputerVisionNode::configurePublishers()
// {
//     // TODO: Check if this is the correct way to do this in C++
//     rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_elevation_mapping_state_pub;
//     rclcpp_lifecycle::LifecyclePublisher<lifecycle_msgs::msg::Transition>::SharedPtr m_plane_segmentation_state_pub;
//     m_elevation_mapping_state_pub = this->create_publisher<lifecycle_msgs::msg::Transition>("/elevation_mapping/transition_event", 10);
//     m_plane_segmentation_state_pub = this->create_publisher<lifecycle_msgs::msg::Transition>("/plane_segmentation/transition_event", 10);
//     m_left_synced_pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cv_left_pc", 10);
//     m_right_synced_pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cv_right_pc", 10);
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
//         m_left_synced_pc_pub->publish(*msg);
//     } else if (msg->header.frame_id == "right_camera") {
//         m_right_synced_pc_pub->publish(*msg);
//     }
// }

// void ComputerVisionNode::dualCameraCallback(
//     const sensor_msgs::msg::PointCloud2::SharedPtr left_msg, 
//     const sensor_msgs::msg::PointCloud2::SharedPtr right_msg)
// {
//     // m_left_camera_interface.processPointCloud(left_msg);
//     // m_right_camera_interface.processPointCloud(right_msg);
//     std::lock_guard<std::mutex> lock(m_mutex);
//     // pointCloudRegistration(m_pc_registration_method, left_msg, right_msg);
//     m_left_synced_pc_pub->publish(*left_msg);
//     m_right_synced_pc_pub->publish(*right_msg);
// }

// // Cybathlon specific setup
// void ComputerVisionNode::exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg)
// {
//     // TODO: Implement switch case / state machine for different modes
//     m_exo_mode = (ExoMode) msg->mode;
//     if (m_exo_mode == ExoMode::Sit) { RCLCPP_INFO(this->get_logger(), "Sit mode detected"); }
//     if (m_exo_mode == ExoMode::Stand) { RCLCPP_INFO(this->get_logger(), "Stand mode detected"); }
//     else { RCLCPP_INFO(this->get_logger(), "Don't care about this mode yet."); }
// }

// }  // namespace march_vision

// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc, argv);
//     // TODO: Multi or single threaded executor?
//     rclcpp::executors::MultiThreadedExecutor executor;
//     std::shared_ptr<ComputerVisionNode> cv_node = std::make_shared<ComputerVisionNode>();
//     rclcpp::spin(cv_node->get_node_base_interface());
//     rclcpp::shutdown();
//     return 0;
// }
