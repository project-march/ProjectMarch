#include "march_state_estimator/state_estimator_node.hpp"

#include <chrono>
#include <memory>
#include <functional>

StateEstimatorNode::StateEstimatorNode()
    : Node("state_estimator_node")
{
    // Create a timer that calls the timerCallback() function every 10 ms
    m_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&StateEstimatorNode::timerCallback, this));

    // Create a subscription to the joint_states topic
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&StateEstimatorNode::jointStateCallback, this, std::placeholders::_1));

    // Create a subscription to the imu topic
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&StateEstimatorNode::imuCallback, this, std::placeholders::_1));

    // Create a publisher for the state_estimation topic
    m_state_estimation_pub = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation", 10);
}

void StateEstimatorNode::timerCallback()
{
    // Publish the state estimation
    publishStateEstimation();
}

void StateEstimatorNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Store the joint state message
    m_joint_state = msg;
}

void StateEstimatorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Store the imu message
    m_imu = msg;
}

void StateEstimatorNode::publishStateEstimation()
{
    // Create a state estimation message
    auto state_estimation_msg = march_shared_msgs::msg::StateEstimation();

    // Fill the message with data
    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "odom";
    state_estimation_msg.joint_state = *m_joint_state;
    state_estimation_msg.imu = *m_imu;

    // Publish the message
    m_state_estimation_pub->publish(state_estimation_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}