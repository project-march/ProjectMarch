#define BOOST_BIND_NO_PLACEHOLDERS

#include "processing/camera_interface.h"

namespace march_vision {

CameraInterface::CameraInterface(rclcpp::Node* node, 
                                 const std::string& left_or_right)
                                : m_node(node),
                                  m_left_or_right(left_or_right)
{
    RCLCPP_INFO(m_node->get_logger(), "CameraInterface for %s camera is initializing...", m_left_or_right.c_str());
    declareParameters();
    readParameters();
    readFilterOptions();
}

CameraInterface::~CameraInterface() 
{
    RCLCPP_WARN(m_node->get_logger(), "CameraInterface for %s camera has been destroyed.", m_left_or_right.c_str());
}

void CameraInterface::declareParameters() 
{
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_node->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // TODO: Change the name of the frame to correct one
    m_frame_id = "camera_" + m_left_or_right + "_frame";
    m_last_frame_time = std::clock();
    m_frame_wait_counter = 0;
    m_frame_timeout = 5.0;
    // TODO: Change the topic name

    m_preprocessed_pointcloud_publisher = m_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cameras/" + m_left_or_right + "/depth/color/points", 10);
    m_realsense_callback_group = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_point_callback_group = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions realsense_callback_options_;
    realsense_callback_options_.callback_group = m_realsense_callback_group;
    rclcpp::SubscriptionOptions point_callback_options_;
    point_callback_options_.callback_group = m_point_callback_group;

    // TODO: Decide on topic
    m_topic_camera = "/cameras/" + m_left_or_right + "/depth/color/points";

    m_config = rs2::config();
    m_pipeline = rs2::pipeline();

    if (m_left_or_right == "left") {

        m_node->declare_parameter("left_camera.serial_number", std::string());
        m_node->declare_parameter("left_camera.resolution.width", 640);
        m_node->declare_parameter("left_camera.resolution.height", 480);
        m_node->declare_parameter("left_camera.fps", 15);

        m_node->declare_parameter("left_camera.realsense_camera_settings.decimation_filter_enabled", false);
        m_node->declare_parameter("left_camera.realsense_camera_settings.spatial_filter_enabled", false);
        m_node->declare_parameter("left_camera.realsense_camera_settings.temporal_filter_enabled", false);
        m_node->declare_parameter("left_camera.realsense_camera_settings.hole_filling_filter_enabled", false);
        m_node->declare_parameter("left_camera.realsense_camera_settings.threshold_filter_enabled", false);
        
        m_node->declare_parameter("left_camera.realsense_camera_settings.decimation_filter_magnitude", 2.0);
        m_node->declare_parameter("left_camera.realsense_camera_settings.spatial_filter_magnitude", 2.0);
        m_node->declare_parameter("left_camera.realsense_camera_settings.spatial_filter_smooth_alpha", 0.5);
        m_node->declare_parameter("left_camera.realsense_camera_settings.spatial_filter_smooth_delta", 20.0);
        m_node->declare_parameter("left_camera.realsense_camera_settings.temporal_filter_smooth_alpha", 0.4);
        m_node->declare_parameter("left_camera.realsense_camera_settings.temporal_filter_smooth_delta", 20.0);
        m_node->declare_parameter("left_camera.realsense_camera_settings.hole_filling_filter_mode", "2d");
        m_node->declare_parameter("left_camera.realsense_camera_settings.threshold_filter_max_distance", 3.5);
        m_node->declare_parameter("left_camera.realsense_camera_settings.threshold_filter_min_distance", 0.9);

    } else if (m_left_or_right == "right") {

        m_node->declare_parameter("right_camera.serial_number", std::string());
        m_node->declare_parameter("right_camera.resolution.width", 640);
        m_node->declare_parameter("right_camera.resolution.height", 480);
        m_node->declare_parameter("right_camera.fps", 15);

        m_node->declare_parameter("right_camera.realsense_camera_settings.decimation_filter_enabled", false);
        m_node->declare_parameter("right_camera.realsense_camera_settings.spatial_filter_enabled", false);
        m_node->declare_parameter("right_camera.realsense_camera_settings.temporal_filter_enabled", false);
        m_node->declare_parameter("right_camera.realsense_camera_settings.hole_filling_filter_enabled", false);
        m_node->declare_parameter("right_camera.realsense_camera_settings.threshold_filter_enabled", false);
        
        m_node->declare_parameter("right_camera.realsense_camera_settings.decimation_filter_magnitude", 2.0);
        m_node->declare_parameter("right_camera.realsense_camera_settings.spatial_filter_magnitude", 2.0);
        m_node->declare_parameter("right_camera.realsense_camera_settings.spatial_filter_smooth_alpha", 0.5);
        m_node->declare_parameter("right_camera.realsense_camera_settings.spatial_filter_smooth_delta", 20.0);
        m_node->declare_parameter("right_camera.realsense_camera_settings.temporal_filter_smooth_alpha", 0.4);
        m_node->declare_parameter("right_camera.realsense_camera_settings.temporal_filter_smooth_delta", 20.0);
        m_node->declare_parameter("right_camera.realsense_camera_settings.hole_filling_filter_mode", "2d");
        m_node->declare_parameter("right_camera.realsense_camera_settings.threshold_filter_max_distance", 3.5);
        m_node->declare_parameter("right_camera.realsense_camera_settings.threshold_filter_min_distance", 0.9);
    }
}

void CameraInterface::readParameters()
{
    if (m_left_or_right == "left") {

        m_serial_number = m_node->get_parameter("left_camera.serial_number").as_string();
        m_resolution_width = m_node->get_parameter("left_camera.resolution.width").as_int();
        m_resolution_height = m_node->get_parameter("left_camera.resolution.height").as_int();
        m_fps = m_node->get_parameter("left_camera.fps").as_int();

        m_decimation_filter_enabled = m_node->get_parameter("left_camera.realsense_camera_settings.decimation_filter_enabled").as_bool();
        m_spatial_filter_enabled = m_node->get_parameter("left_camera.realsense_camera_settings.spatial_filter_enabled").as_bool();
        m_temporal_filter_enabled = m_node->get_parameter("left_camera.realsense_camera_settings.temporal_filter_enabled").as_bool();
        m_hole_filling_filter_enabled = m_node->get_parameter("left_camera.realsense_camera_settings.hole_filling_filter_enabled").as_bool();
        m_threshold_filter_enabled = m_node->get_parameter("left_camera.realsense_camera_settings.threshold_filter_enabled").as_bool();

        m_decimation_filter_magnitude = m_node->get_parameter("left_camera.realsense_camera_settings.decimation_filter_magnitude").as_double();
        m_spatial_filter_magnitude = m_node->get_parameter("left_camera.realsense_camera_settings.spatial_filter_magnitude").as_double();
        m_spatial_filter_smooth_alpha = m_node->get_parameter("left_camera.realsense_camera_settings.spatial_filter_smooth_alpha").as_double();
        m_spatial_filter_smooth_delta = m_node->get_parameter("left_camera.realsense_camera_settings.spatial_filter_smooth_delta").as_double();
        m_temporal_filter_smooth_alpha = m_node->get_parameter("left_camera.realsense_camera_settings.temporal_filter_smooth_alpha").as_double();
        m_temporal_filter_smooth_delta = m_node->get_parameter("left_camera.realsense_camera_settings.temporal_filter_smooth_delta").as_double();
        m_hole_filling_filter_mode = m_node->get_parameter("left_camera.realsense_camera_settings.hole_filling_filter_mode").as_string();
        m_threshold_filter_max_distance = m_node->get_parameter("left_camera.realsense_camera_settings.threshold_filter_max_distance").as_double();
        m_threshold_filter_min_distance = m_node->get_parameter("left_camera.realsense_camera_settings.threshold_filter_min_distance").as_double();

    } else if (m_left_or_right == "right") {

        m_serial_number = m_node->get_parameter("right_camera.serial_number").as_string();
        m_resolution_width = m_node->get_parameter("right_camera.resolution.width").as_int();
        m_resolution_height = m_node->get_parameter("right_camera.resolution.height").as_int();
        m_fps = m_node->get_parameter("right_camera.fps").as_int();

        m_decimation_filter_enabled = m_node->get_parameter("right_camera.realsense_camera_settings.decimation_filter_enabled").as_bool();
        m_spatial_filter_enabled = m_node->get_parameter("right_camera.realsense_camera_settings.spatial_filter_enabled").as_bool();
        m_temporal_filter_enabled = m_node->get_parameter("right_camera.realsense_camera_settings.temporal_filter_enabled").as_bool();
        m_hole_filling_filter_enabled = m_node->get_parameter("right_camera.realsense_camera_settings.hole_filling_filter_enabled").as_bool();

        m_decimation_filter_magnitude = m_node->get_parameter("right_camera.realsense_camera_settings.decimation_filter_magnitude").as_double();
        m_spatial_filter_magnitude = m_node->get_parameter("right_camera.realsense_camera_settings.spatial_filter_magnitude").as_double();
        m_spatial_filter_smooth_alpha = m_node->get_parameter("right_camera.realsense_camera_settings.spatial_filter_smooth_alpha").as_double();
        m_spatial_filter_smooth_delta = m_node->get_parameter("right_camera.realsense_camera_settings.spatial_filter_smooth_delta").as_double();
        m_temporal_filter_smooth_alpha = m_node->get_parameter("right_camera.realsense_camera_settings.temporal_filter_smooth_alpha").as_double();
        m_temporal_filter_smooth_delta = m_node->get_parameter("right_camera.realsense_camera_settings.temporal_filter_smooth_delta").as_double();
        m_hole_filling_filter_mode = m_node->get_parameter("right_camera.realsense_camera_settings.hole_filling_filter_mode").as_string();
        m_threshold_filter_max_distance = m_node->get_parameter("right_camera.realsense_camera_settings.threshold_filter_max_distance").as_double();
        m_threshold_filter_min_distance = m_node->get_parameter("right_camera.realsense_camera_settings.threshold_filter_min_distance").as_double();
    }
}

void CameraInterface::readFilterOptions()
{
    if (m_decimation_filter_enabled) {
        m_decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, m_decimation_filter_magnitude);
    }

    if (m_spatial_filter_enabled) {
        m_spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, m_spatial_filter_magnitude);
        m_spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, m_spatial_filter_smooth_alpha);
        m_spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, m_spatial_filter_smooth_delta);
    }

    if (m_temporal_filter_enabled) {
        m_temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, m_temporal_filter_smooth_alpha);
        m_temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, m_temporal_filter_smooth_delta);
    }

    if (m_hole_filling_filter_enabled) {
        if (m_hole_filling_filter_mode == "fill") { // "fill"
            m_hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 1);
        } else if (m_hole_filling_filter_mode == "2d") { // "2d"
            m_hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 2);
        }
    }
    if (m_threshold_filter_enabled) {
        m_threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, m_threshold_filter_min_distance);
        m_threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, m_threshold_filter_max_distance);
    }
}

bool CameraInterface::initializeCamera() 
{
    while (true) {
        try {
            RCLCPP_INFO(m_node->get_logger(), "Looking for %s cam with serial number: %s", m_left_or_right.c_str(), m_serial_number.c_str());
            m_config.enable_device(m_serial_number);
            // TODO: Change the resolution and frame rate
            m_config.enable_stream(RS2_STREAM_DEPTH, m_resolution_width, m_resolution_height, RS2_FORMAT_Z16, m_fps);
            m_pipeline.start(m_config);
            
            // Setup successful, create timer and log success
            m_realsense_timer = m_node->create_wall_timer(
                std::chrono::milliseconds(30),
                [this]() -> void { processRealSenseDepthFrames(); }, m_realsense_callback_group);
            
            RCLCPP_INFO(m_node->get_logger(), "\033[1;36m%s RealSense connected (%s) \033[0m", 
                        m_left_or_right.c_str(), m_serial_number.c_str());

            return true;  
        } catch (const rs2::error& e) {
            // Log initialization error
            std::string error_message = e.what();
            RCLCPP_WARN(m_node->get_logger(), "Error while initializing %s RealSense camera: %s",
                        m_left_or_right.c_str(), error_message.c_str());
            
            rclcpp::sleep_for(std::chrono::nanoseconds(1000000000)); // Wait for 1 second before retrying
            continue;
        }
    }
    return false;
}

void CameraInterface::processRealSenseDepthFrames() 
{
    float difference = float(std::clock() - m_last_frame_time) / CLOCKS_PER_SEC;
    if ((int)(difference / m_frame_timeout) > m_frame_wait_counter) {
        m_frame_wait_counter++;
        RCLCPP_WARN(m_node->get_logger(), "RealSense (%s) did not receive frames last %d seconds", m_left_or_right.c_str(),
            m_frame_wait_counter * (int) m_frame_timeout);
    }

    rs2::frameset frames = m_pipeline.wait_for_frames();
    rs2::depth_frame depth_frame = frames.get_depth_frame();

    // NOTE: Order matters
    if (m_decimation_filter_enabled){ depth_frame = m_decimation_filter.process(depth_frame);}
    if (m_spatial_filter_enabled){ depth_frame = m_spatial_filter.process(depth_frame);}
    if (m_temporal_filter_enabled){ depth_frame = m_temporal_filter.process(depth_frame);}
    if (m_hole_filling_filter_enabled){ depth_frame = m_hole_filling_filter.process(depth_frame);}
    if (m_threshold_filter_enabled){ depth_frame = m_threshold_filter.process(depth_frame);}

    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth_frame);
    PointCloud::Ptr point_cloud = pointsToPCL(points);

    m_frame_time = depth_frame.get_timestamp();
    RCLCPP_INFO(m_node->get_logger(), "%s cam | frame_time: %f", m_left_or_right.c_str(), m_frame_time);
    point_cloud->header.frame_id = m_frame_id;
    processPointCloud(point_cloud);
}

PointCloud::Ptr CameraInterface::pointsToPCL(const rs2::points& points) 
{
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

void CameraInterface::processPointCloud(const PointCloud::Ptr& pointcloud) 
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_last_frame_time = std::clock();
    m_frame_wait_counter = 0;
    publishCloud(*pointcloud);
}

void CameraInterface::publishCloud(PointCloud cloud) 
{
    cloud.width = 1;
    cloud.height = cloud.points.size();
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = m_frame_id;
    msg.header.stamp = m_node->now();
    
    // RCLCPP_INFO(m_node->get_logger(), "PC from %s camera published at %f", m_left_or_right.c_str(), msg.header.stamp.nanosec);
    m_preprocessed_pointcloud_publisher->publish(msg);
}

void CameraInterface::shutdown() 
{
    m_pipeline.stop();
    m_tf_buffer.reset();
    m_tf_listener.reset();
    if (m_realsense_timer) {
        m_realsense_timer->cancel();
        m_realsense_timer.reset();
    }
    m_realsense_callback_group.reset();
    m_point_callback_group.reset();

    RCLCPP_INFO(m_node->get_logger(), "CameraInterface for %s camera has been shut down.", m_left_or_right.c_str());
}

void CameraInterface::run() 
{
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(m_node->get_node_base_interface());
    // executor.spin();
    RCLCPP_INFO(m_node->get_logger(), "CameraInterface for %s camera has been started.", m_left_or_right.c_str());
}

void CameraInterface::stop() 
{
    // executor.cancel();
    RCLCPP_INFO(m_node->get_logger(), "CameraInterface for %s camera has been stopped.", m_left_or_right.c_str());
}
}