/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/sensor_fusion_node.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <unordered_map>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "math.h"

using std::placeholders::_1;
using std::placeholders::_2;

SensorFusionNode::SensorFusionNode(std::shared_ptr<RobotDescription> robot_description)
    : Node("state_estimator_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing State Estimator Node");

    // this->declare_parameter<int64_t>("dt", 50);
    // int64_t dt = this->get_parameter("dt").as_int();
    int64_t dt = 50;

    m_sensors_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_sensors_subscription_options.callback_group = m_sensors_callback_group;

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(dt), std::bind(&SensorFusionNode::timerCallback, this), m_sensors_callback_group);
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::jointStateCallback, this, std::placeholders::_1),
        m_sensors_subscription_options);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("lower_imu", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_state_estimation_pub
        = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10);
    m_filtered_imu_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("filtered_imu", 10);

    std::vector<std::string> joint_names = { "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa",
        "right_hip_fe", "right_knee", "right_ankle" };
    m_sensor_fusion = std::make_unique<SensorFusion>(robot_description);
    m_sensor_fusion->configureJointNames(joint_names);
    m_robot_description = robot_description;
    m_dt = static_cast<double>(dt) / 1000.0;
    m_joint_state = nullptr;
    m_imu = nullptr;
    m_node_feet_names = { "L_foot", "R_foot" };

    RCLCPP_INFO(this->get_logger(), "State Estimator Node initialized");
}

SensorFusionNode::~SensorFusionNode()
{
    RCLCPP_WARN(rclcpp::get_logger("march_state_estimator"), "StateEstimatorNode has been stopped.");
}

void SensorFusionNode::timerCallback()
{
    // if (m_joint_state == nullptr || m_imu == nullptr) {
    if (m_joint_state == nullptr || m_imu == nullptr) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No joint state or imu data received yet");
        return;
    }
    publishStateEstimation();
}

void SensorFusionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    m_joint_state = msg;
    m_sensor_fusion->updateJointState(m_joint_state);
}

void SensorFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu = msg;
    m_sensor_fusion->updateImu(m_imu);
    if (m_joint_state != nullptr) {
        m_sensor_fusion->updateKalmanFilter();
    }
}

void SensorFusionNode::publishStateEstimation()
{
    march_shared_msgs::msg::StateEstimation state_estimation_msg;
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes(m_node_feet_names);

    // Find a way to optimize this
    std::unordered_map<std::string, double> joint_positions;
    for (long unsigned int i = 0; i < m_joint_state->name.size(); i++) {
        joint_positions[m_joint_state->name[i]] = m_joint_state->position[i];
    }

    std::vector<geometry_msgs::msg::Pose> foot_poses = m_sensor_fusion->getFootPoses();
    uint8_t stance_leg = m_sensor_fusion->updateStanceLeg(&foot_poses[0].position, &foot_poses[1].position);

    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "backpack";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = *m_joint_state;
    state_estimation_msg.imu = *m_sensor_fusion->getFilteredImuMsg();
    state_estimation_msg.foot_pose = foot_poses;
    state_estimation_msg.stance_leg = stance_leg;
    // state_estimation_msg.zmp = m_sensor_fusion->getZMP();
    state_estimation_msg.imu_orientation = *m_sensor_fusion->getFilteredOrientationMsg();
    m_state_estimation_pub->publish(state_estimation_msg);

    geometry_msgs::msg::TransformStamped filtered_imu_msg;
    filtered_imu_msg.header.stamp = this->now();
    filtered_imu_msg.header.frame_id = "map";
    filtered_imu_msg.child_frame_id = "backpack";
    filtered_imu_msg.transform.translation.x = m_sensor_fusion->getFilteredImuMsg()->linear_acceleration.x;
    filtered_imu_msg.transform.translation.y = m_sensor_fusion->getFilteredImuMsg()->linear_acceleration.y;
    filtered_imu_msg.transform.translation.z = m_sensor_fusion->getFilteredImuMsg()->linear_acceleration.z;
    filtered_imu_msg.transform.rotation = m_sensor_fusion->getFilteredImuMsg()->orientation;
    m_filtered_imu_pub->publish(filtered_imu_msg);
}