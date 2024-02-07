#include "march_state_estimator/transform_publisher_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

TransformPublisherNode::TransformPublisherNode()
    : Node("transform_publisher")
{
    declare_parameter<int>("refresh_rate", 100);
    int refresh_rate = get_parameter("refresh_rate").as_int();

    m_timer = create_wall_timer(
        std::chrono::milliseconds(1000 / refresh_rate), std::bind(&TransformPublisherNode::publishTransforms, this));

    // m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void TransformPublisherNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    m_joint_state = std::make_shared<sensor_msgs::msg::JointState>(msg->joint_state);
}

void TransformPublisherNode::publishTransforms()
{
    if (m_joint_state) {
        for (size_t i = 0; i < m_joint_state->name.size(); i++) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = m_joint_state->header.stamp;
            transform_stamped.header.frame_id = "hip_base";
            transform_stamped.child_frame_id = m_joint_state->name[i];
            transform_stamped.transform.translation.x = m_joint_state->position[i];
            transform_stamped.transform.rotation.w = 1;
            m_tf_broadcaster->sendTransform(transform_stamped);
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}