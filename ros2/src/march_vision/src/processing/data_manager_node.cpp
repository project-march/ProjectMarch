#include "march_vision/processing/input_source_manager_node.hpp"


InputSourceManagerNode::InputSourceManagerNode() : Node("input_source_manager_node")
{
    m_left_camera_interface = CameraInterface(this, "left");
    m_right_camera_interface = CameraInterface(this, "right");
    
    m_left_camera_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/cameras_left/depth/color/points", 100);
    m_right_camera_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/cameras_right/depth/color/points", 100);
    
    m_sync = std::make_shared<message_filters::Synchronizer<m_sync_policy>>(m_sync_policy(100), *m_left_camera_sub, *m_right_camera_sub);
    m_sync->registerCallback(std::bind(&InputSourceManagerNode::dualCameraCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void InputSourceManagerNode::dualCameraCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr left_msg, 
    const sensor_msgs::msg::PointCloud2::SharedPtr right_msg)
{
    m_left_camera_interface.processPointCloud(left_msg);
    m_right_camera_interface.processPointCloud(right_msg);
}