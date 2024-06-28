#include "march_vision/processing/camera_interface.h"
// TODO: Change to new msg for current state
// #include "march_shared_msgs/msg/current_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
// TODO: Include realsense2 sdk

CameraInterface::CameraInterface(rclcpp::Node* node, 
                                const std::string& left_or_right)
                                : m_node(node),
                                  m_left_or_right(left_or_right),
                                  m_realsense_camera_settings(realsense_camera_settings)
{
    RCLCPP_INFO(m_node->get_logger(), "CameraInterface for %s camera is initializing...", m_left_or_right.c_str());
    declareParameters();
}

CameraInterface::~CameraInterface() 
{
    RCLCPP_WARN(m_node->get_logger(), "CameraInterface for %s camera has been destroyed.", m_left_or_right.c_str());
}

void CameraInterface::declareParameters() 
{
    m_serial_number = "000000000000";

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_node->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // TODO: Camera frame or body frame?
    m_frame_id = "camera_" + m_left_or_right + "_frame";
    m_last_frame_time = std::clock();
    m_frame_wait_counter = 0;
    m_frame_timeout = 5.0;

    m_realsense_callback_group = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_point_callback_group = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions realsense_callback_options_;
    realsense_callback_options_.callback_group = m_realsense_callback_group;

    rclcpp::SubscriptionOptions point_callback_options_;
    point_callback_options_.callback_group = m_point_callback_group;

    // TODO: Decide on topic
    m_topic_camera = "/cameras/" + left_or_right + "/depth/color/points";
    m_node->declare_parameter("left_camera_serial_number", int());


    m_config = rs2::config();
    m_pipe = rs2::pipeline();
    // TODO: Read it from here or get it passed through CV node?
    // m_node->declare_parameter("left_camera_serial_number", int());
    // declare_parameter("right_camera_serial_number", int());
    // // Realsense cameras settings
    // declare_parameter("realsense_camera_settings.decimation_filter", bool());
    // declare_parameter("realsense_camera_settings.spatial_filter", bool());
    // declare_parameter("realsense_camera_settings.temporal_filter", bool());
    // declare_parameter("realsense_camera_settings.hole_filling_filter", bool());
    // declare_parameter("realsense_camera_settings.threshold_filter", bool());
}

void CameraInterface::readParameters() 
{
    // TODO: Read it from here or get it passed through CV node?
}

void CameraInterface::initializeCamera(const std::string& serial_number) 
{
    while (true) {
        try {
            m_config.enable_device(serial_number);
            m_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
            m_pipeline.start(m_config);
        } catch (const rs2::error& e) {
            std::string error_message = e.what();
            RCLCPP_WARN(m_node->get_logger(), "Error while initializing %s RealSense camera: %s",
                m_left_or_right.c_str(), error_message.c_str());
            rclcpp::sleep_for(std::chrono::nanoseconds(1000000000)); // 1 second
            continue;
        }
        m_realsense_timer = m_node->create_wall_timer(
            std::chrono::milliseconds(30),
            [this]() -> void { processRealSenseDepthFrames();}, m_realsense_callback_group);
        RCLCPP_INFO(m_node->get_logger(), "\033[1;36m%s RealSense connected (%s) \033[0m", m_left_or_right.c_str(),
        m_serial_number.c_str());
        break;
    }
}

void CameraInterface::processRealSenseDepthFrames() {

    float difference = float(std::clock() - m_last_frame_time) / CLOCKS_PER_SEC;

    if ((int)(difference / m_frameTimeout) > m_frame_wait_counter) {
        m_frame_wait_counter++;
        RCLCPP_WARN(m_node->get_logger(), "RealSense (%s) did not receive frames last %d seconds", m_left_or_right.c_str(),
            m_frame_wait_counter * (int) m_frame_timeout);
    }

    rs2::frameset frames = m_pipe.wait_for_frames();
    rs2::depth_frame depth_frame = frames.get_depth_frame();

    // NOTE: Order matters
    depth_frame = m_dec_filter.process(depth_frame);
    depth_frame = m_spat_filter.process(depth_frame);
    depth_frame = m_temp_filter.process(depth_frame);

    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth_frame);
    PointCloud::Ptr point_cloud = pointsToPCL(points);

    // TODO: Change the topic?
    //point_cloud->header.frame_id = "camera_" + m_left_or_right + "_depth_optical_frame";
    processPointCloud(point_cloud);
}

// TODO: M7 waits for 1s ?
void CameraInterface::processPointCloud(const PointCloud::Ptr& pointcloud) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_last_frame_time = std::clock();
    m_frame_wait_counter = 0;
    publishCloud(m_preprocessed_pointcloud_publisher, m_node, *pointcloud, m_left_or_right);
}

PointCloud::Ptr CameraInterface::pointsToPCL(const rs2::points& points) {

    PointCloud::Ptr cloud = boost::make_shared<PointCloud>();

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    int point_count = 0;
    auto ptr = points.get_vertices();
    for (std::size_t i = 0; i < points.size(); i++) {
        (*cloud)[point_count].x = ptr->x;
        (*cloud)[point_count].y = ptr->y;
        (*cloud)[point_count].z = ptr->z;   
        point_count++;
        ptr++;
    }
    cloud->points.resize(point_count);
    return cloud;
}

void CameraInterface::publishCloud(
    const PointCloudPublisher::SharedPtr& publisher, rclcpp::Node* n, PointCloud cloud, std::string& left_or_right)
{
    cloud.width = 1;
    cloud.height = cloud.points.size();
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "/cameras/" + left_or_right + "/depth/color/points";

    // TODO: Change to use the timeframe when the point cloud was received.
    msg.header.stamp = n->now();
    publisher->publish(msg);
}

// void CameraInterface::shutdown() {

//     m_pipeline.stop();

//     m_tf_buffer.reset();
//     m_tf_listener.reset();

//     if (m_realsense_timer) {
//         m_realsense_timer->cancel();
//         m_realsense_timer.reset();
//     }

//     m_realsense_callback_group.reset();
//     m_point_callback_group.reset();

////     RCLCPP_INFO(m_node->get_logger(), "CameraInterface for %s camera has been shut down.", m_left_or_right.c_str());
// }