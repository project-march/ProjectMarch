#include "march_vision/processing/camera_interface.h"
// TODO: Change to new msg for current state
//#include "march_shared_msgs/msg/current_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

// TODO: Fix this message
//#include <visualization_msgs/msg/marker_array.hpp>

CameraInterface::CameraInterface(rclcpp::Node* n, const std::string& left_or_right)
    : m_node(n)
    , m_left_or_right(left_or_right)
{
    // TODO: Change this to a config or launch param?
    m_serial_number = (m_left_or_right == "left") ? "944622074337" : "944622071535";

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(m_node->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // TODO: Delete when getting backpack
    m_ORIGIN = Point(0, 0, 0);
    m_last_frame_time = std::clock();
    m_frame_wait_counter = 0;
    m_frame_timeout = 5.0;

    m_realsense_callback_rgoup = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_point_callback_group = m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions realsense_callback_options_;
    realsense_callback_options_.callback_group = m_realsense_callback_group;

    rclcpp::SubscriptionOptions point_callback_options_;
    point_callback_options_.callback_group = m_point_callback_group;

    // TODO: Decide on topic
    m_topic_camera = "/cameras" + left_or_right + "/depth/color/points";

    m_sample_size = m_node->get_parameter("sample_size").as_int();
    m_outlier_distance = m_node->get_parameter("outlier_distance").as_double();
    m_height_zero_threshold = m_node->get_parameter("height_zero_threshold").as_double();
    m_height_distance_coefficient = m_node->get_parameter("height_distance_coefficient").as_double();
    m_realsense_simulation = m_node->get_parameter("realsense_simulation").as_bool();

    // TODO: What did they use this for?
    //found_points_.resize(m_sampleSize);

    if (!m_realsense_simulation) {

        while (true) {
            try {
                m_config.enable_device(m_serial_number);
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
                [this]() -> void {
                    processRealSenseDepthFrames();
                },
                m_realsense_callback_group);
            RCLCPP_INFO(m_node->get_logger(), "\033[1;36m%s RealSense connected (%s) \033[0m", m_left_or_right.c_str(),
                m_serial_number.c_str());

            break;
        }
    } else {
        return;
    }
}

void CameraInterface::readParameters(const std::vector<rclcpp::Parameter>& parameters) {

    for (const auto& param : parameters) {
        const std::string& name = param.get_name();
        
        if (name == "realsense_simulation") {
            m_realsense_simulation = param.as_bool();
        } else if (name == "sample_size") {
            m_sample_size = param.as_int();
        } else if (name == "outlier_distance") {
            m_outlier_distance = param.as_double();
        } else if (name == "height_zero_threshold") {
            m_height_zero_threshold = param.as_double();
        } else if (name == "height_distance_coefficient") {
            m_height_distance_coefficient = param.as_double();
        } else {
            break;
        }
        RCLCPP_INFO(m_node->get_logger(), "\033[92mParameter %s updated in %s Camera Interface\033[0m",
            param.get_name().c_str(), m_left_or_right.c_str());
    }
    //found_points_.resize(m_sample_size);
}

void CameraInterface::processRealSenseDepthFrames() {

    float difference = float(std::clock() - m_last_frame_time) / CLOCKS_PER_SEC;

    if ((int)(difference / m_frameTimeout) > m_frame_wait_counter) {
        m_frame_wait_counter++;
        RCLCPP_WARN(m_node->get_logger(), "RealSense (%s) did not receive frames last %d seconds", m_left_or_right.c_str(),
            m_frame_wait_counter * (int) m_frame_timeout);
    }

    rs2::frameset frames = m_pipe.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    depth = m_dec_filter.process(depth);
    depth = m_spat_filter.process(depth);
    depth = m_temp_filter.process(depth);

    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth);
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