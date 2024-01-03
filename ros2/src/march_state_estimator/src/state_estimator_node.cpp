#include "march_state_estimator/state_estimator_node.hpp"

#include <chrono>
#include <memory>
#include <functional>

StateEstimatorNode::StateEstimatorNode()
    : Node("state_estimator_node")
{
    // Declare the parameters
    this->declare_parameter<int64_t>("dt", 50);

    // Get the parameters
    int64_t dt = this->get_parameter("dt").as_int();
    m_dt = static_cast<double>(dt) / 1000.0;

    m_timer = this->create_wall_timer(std::chrono::milliseconds(dt), std::bind(&StateEstimatorNode::timerCallback, this));
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&StateEstimatorNode::jointStateCallback, this, std::placeholders::_1));
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&StateEstimatorNode::imuCallback, this, std::placeholders::_1));
    m_state_estimation_pub = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10);

    sensor_msgs::msg::JointState init_jointstate_msg;
    // Set the initial joint state message to zero data
    init_jointstate_msg.header.stamp = this->now();
    init_jointstate_msg.header.frame_id = "backpack";
    init_jointstate_msg.name = {};
    init_jointstate_msg.position = {};
    init_jointstate_msg.velocity = {};
    init_jointstate_msg.effort = {};

    m_joint_state = std::make_shared<sensor_msgs::msg::JointState>(init_jointstate_msg);

    sensor_msgs::msg::Imu init_imu_msg;
    // Set the initial imu message to zero data
    init_imu_msg.header.stamp = this->now();
    init_imu_msg.header.frame_id = "backpack";
    init_imu_msg.orientation.x = 0.0;
    init_imu_msg.orientation.y = 0.0;
    init_imu_msg.orientation.z = 0.0;
    init_imu_msg.orientation.w = 1.0;
    init_imu_msg.angular_velocity.x = 0.0;
    init_imu_msg.angular_velocity.y = 0.0;
    init_imu_msg.angular_velocity.z = 0.0;
    init_imu_msg.linear_acceleration.x = 0.0;
    init_imu_msg.linear_acceleration.y = 0.0;
    init_imu_msg.linear_acceleration.z = 0.0;

    m_imu = std::make_shared<sensor_msgs::msg::Imu>(init_imu_msg);

    RCLCPP_INFO(this->get_logger(), "State Estimator Node initialized");
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
    march_shared_msgs::msg::StateEstimation state_estimation_msg;

    // Fill the message with data
    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "backpack";
    state_estimation_msg.step_time = m_dt;
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