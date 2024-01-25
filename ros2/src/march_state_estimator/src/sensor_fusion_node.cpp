/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/sensor_fusion_node.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <future>
#include <unordered_map>

#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

using std::placeholders::_1;
using std::placeholders::_2;

SensorFusionNode::SensorFusionNode(std::shared_ptr<RobotDescription> robot_description)
    : Node("state_estimator_node")
{
    // this->declare_parameter<int64_t>("dt", 50);
    // int64_t dt = this->get_parameter("dt").as_int();
    int64_t dt = 50;

    m_joint_state_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_imu_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_timer_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_node_position_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_joint_state_subscription_options.callback_group = m_joint_state_callback_group;
    m_imu_subscription_options.callback_group = m_imu_callback_group;

    m_timer = this->create_wall_timer(std::chrono::milliseconds(dt), std::bind(&SensorFusionNode::timerCallback, this), m_timer_callback_group);
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), std::bind(&SensorFusionNode::jointStateCallback, this, std::placeholders::_1), m_joint_state_subscription_options);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1), m_imu_subscription_options);
    m_state_estimation_pub = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 1);

    sensor_msgs::msg::Imu init_imu_msg;
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

    m_robot_description = robot_description;
    m_dt = static_cast<double>(dt) / 1000.0;
    m_joint_state = nullptr;
    m_imu = std::make_shared<sensor_msgs::msg::Imu>(init_imu_msg);
    m_node_feet_names = {"L_foot", "R_foot"};

    RCLCPP_INFO(this->get_logger(), "State Estimator Node initialized");
}

void SensorFusionNode::timerCallback()
{
    if (m_joint_state == nullptr)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No joint state received yet");
        return;
    }

    publishStateEstimation();
}

void SensorFusionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    m_joint_state = msg;
}

void SensorFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu = msg;
}

void SensorFusionNode::publishStateEstimation()
{
    march_shared_msgs::msg::StateEstimation state_estimation_msg;

    uint8_t stance_leg = 0b11;
    std::vector<geometry_msgs::msg::Pose> foot_poses;
    std::vector<std::shared_ptr<RobotNode>> feet_nodes = m_robot_description->findNodes(m_node_feet_names);

    // Find a way to optimize this
    std::unordered_map<std::string, double> joint_positions;
    for (long unsigned int i = 0; i < m_joint_state->name.size(); i++)
    {
        joint_positions[m_joint_state->name[i]] = m_joint_state->position[i];
    }

    for (long unsigned int i = 0; i < feet_nodes.size(); i++)
    {
        geometry_msgs::msg::Pose foot_pose;
        Eigen::Vector3d foot_position = feet_nodes[i]->getGlobalPosition(joint_positions);
        foot_pose.position.x = foot_position.x();
        foot_pose.position.y = foot_position.y();
        foot_pose.position.z = foot_position.z();
        foot_pose.orientation.x = 0.0;
        foot_pose.orientation.y = 0.0;
        foot_pose.orientation.z = 0.0;
        foot_pose.orientation.w = 1.0;
        foot_poses.push_back(foot_pose);    
    }

    double margin = 0.005;
    if (abs(foot_poses[0].position.x - foot_poses[1].position.x) <= margin)
    {
        stance_leg = 0b11;
    }
    else if (foot_poses[0].position.x + margin <= foot_poses[1].position.x)
    {
        stance_leg = 0b01;
    }
    else if (foot_poses[0].position.x - margin > foot_poses[1].position.x)
    {
        stance_leg = 0b10;
    }

    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "backpack";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = *m_joint_state;
    state_estimation_msg.imu = *m_imu;
    state_estimation_msg.foot_pose = foot_poses;
    state_estimation_msg.stance_leg = stance_leg;
    m_state_estimation_pub->publish(state_estimation_msg);
}

void SensorFusionNode::handleGetCurrentJointPositions(std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request>,
    std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response)
{
    response->joint_positions = m_joint_state->position;
}