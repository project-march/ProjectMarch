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

    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_sensors_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_sensors_subscription_options.callback_group = m_sensors_callback_group;

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(dt), std::bind(&SensorFusionNode::timerCallback, this), m_sensors_callback_group);
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::jointStateCallback, this, std::placeholders::_1),
        m_sensors_subscription_options);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("lower_imu", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_imu_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("lower_imu/position", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuPositionCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_state_estimation_pub
        = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10);
    m_feet_height_pub 
        = this->create_publisher<march_shared_msgs::msg::FeetHeightStamped>("state_estimation/feet_height", 10);

    // M8's MPC
    m_mpc_foot_positions_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("est_foot_position", 10);
    m_mpc_com_pub = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 10);
    m_mpc_zmp_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 10);
    m_mpc_stance_foot_pub = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 10);

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
    if (m_joint_state == nullptr || m_imu == nullptr) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No joint state or imu data received yet");
        return;
    }

    if (m_imu_position == nullptr) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No imu position data received yet");
        return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "backpack";
    // transform_stamped.transform = m_sensor_fusion->getRobotTransform();
    transform_stamped.transform.translation.x = m_imu_position->point.x;
    transform_stamped.transform.translation.y = m_imu_position->point.y;
    transform_stamped.transform.translation.z = m_imu_position->point.z;
    transform_stamped.transform.rotation.x = m_imu->orientation.x;
    transform_stamped.transform.rotation.y = m_imu->orientation.y;
    transform_stamped.transform.rotation.z = m_imu->orientation.z;
    transform_stamped.transform.rotation.w = m_imu->orientation.w;
    m_tf_broadcaster->sendTransform(transform_stamped);

    publishStateEstimation();
    publishFeetHeight();
    publishMPCEstimation();
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

    // if (m_joint_state != nullptr) {
    //     m_sensor_fusion->updateKalmanFilter();
    // }
}

void SensorFusionNode::imuPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_imu_position = msg;
}

void SensorFusionNode::publishStateEstimation()
{
    march_shared_msgs::msg::StateEstimation state_estimation_msg;
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes(m_node_feet_names);

    std::vector<geometry_msgs::msg::Pose> foot_poses = m_sensor_fusion->getFootPoses();
    uint8_t stance_leg = m_sensor_fusion->updateStanceLeg(&foot_poses[0].position, &foot_poses[1].position);

    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "backpack";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = *m_joint_state;
    // state_estimation_msg.imu = *m_sensor_fusion->getFilteredImuMsg();
    state_estimation_msg.imu = *m_imu;
    state_estimation_msg.foot_pose = foot_poses;
    state_estimation_msg.stance_leg = stance_leg;
    m_state_estimation_pub->publish(state_estimation_msg);
}

void SensorFusionNode::publishFeetHeight()
{
    march_shared_msgs::msg::FeetHeightStamped feet_height_msg;
    feet_height_msg.header.stamp = this->now();
    feet_height_msg.header.frame_id = "world";
    feet_height_msg.heights = m_sensor_fusion->getFootContactHeight();
    m_feet_height_pub->publish(feet_height_msg);
}

void SensorFusionNode::publishMPCEstimation()
{
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes(m_node_feet_names);
    std::vector<geometry_msgs::msg::Pose> foot_poses = m_sensor_fusion->getFootPoses();
    uint8_t stance_leg = m_sensor_fusion->updateStanceLeg(&foot_poses[0].position, &foot_poses[1].position);

    Eigen::Vector3d com_position = m_sensor_fusion->getCOM() + Eigen::Vector3d(m_imu_position->point.x, m_imu_position->point.y, m_imu_position->point.z);
    Eigen::Vector3d com_velocity = m_sensor_fusion->getCOMVelocity();

    const double gravity = 9.81;
    double zmp_x = com_position.x() + (com_velocity.x() / sqrt(gravity / com_position.z())) * com_position.y();
    double zmp_y = com_position.y() + (com_velocity.y() / sqrt(gravity / com_position.z())) * com_position.z();

    geometry_msgs::msg::PoseArray foot_positions_msg;
    foot_positions_msg.header.stamp = this->now();
    foot_positions_msg.header.frame_id = "world";
    foot_positions_msg.poses = foot_poses;
    m_mpc_foot_positions_pub->publish(foot_positions_msg);

    march_shared_msgs::msg::CenterOfMass com_msg;
    com_msg.header.stamp = this->now();
    com_msg.header.frame_id = "world";
    com_msg.position.x = com_position.x();
    com_msg.position.y = com_position.y();
    com_msg.position.z = com_position.z();
    com_msg.velocity.x = com_velocity.x();
    com_msg.velocity.y = com_velocity.y();
    com_msg.velocity.z = com_velocity.z();
    m_mpc_com_pub->publish(com_msg);

    geometry_msgs::msg::PointStamped zmp_msg;
    zmp_msg.header.stamp = this->now();
    zmp_msg.header.frame_id = "world";
    zmp_msg.point.x = zmp_x;
    zmp_msg.point.y = zmp_y;
    zmp_msg.point.z = 0.0;
    m_mpc_zmp_pub->publish(zmp_msg);

    std_msgs::msg::Int32 stance_foot_msg;
    if (stance_leg == 0b11)
        stance_foot_msg.data = 0;
    else if (stance_leg == 0b10)
        stance_foot_msg.data = -1;
    else if (stance_leg == 0b01)
        stance_foot_msg.data = 1;
    m_mpc_stance_foot_pub->publish(stance_foot_msg);
}