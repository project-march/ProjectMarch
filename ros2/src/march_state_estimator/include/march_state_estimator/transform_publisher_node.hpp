#ifndef MARCH_STATE_ESTIMATOR__TRANSFORM_PUBLISHER_HPP_
#define MARCH_STATE_ESTIMATOR__TRANSFORM_PUBLISHER_HPP_

/*
 *  Author: Alexander James Becoy, alexanderjames.becoy@projectmarch.nl
 *  Date: 2023-01-15
 */

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class TransformPublisherNode : public rclcpp::Node {
public:
    TransformPublisherNode();
    ~TransformPublisherNode() = default;

private:
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void publishTransforms();

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;

    sensor_msgs::msg::JointState::SharedPtr m_joint_state;
    // std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

#endif // MARCH_STATE_ESTIMATOR__TRANSFORM_PUBLISHER_HPP_