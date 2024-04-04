/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/sensor_fusion_node.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
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

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

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
    m_imu_velocity_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("lower_imu/velocity", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuVelocityCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_state_estimation_pub
        = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10);
    m_feet_height_pub 
        = this->create_publisher<march_shared_msgs::msg::FeetHeightStamped>("state_estimation/feet_height", 10);

    // M8's MPC
    m_mpc_foot_positions_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("est_foot_position", 10);
    m_mpc_com_pub = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 10);
    m_mpc_zmp_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 10);
    m_mpc_stance_foot_pub = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 10);

    m_mpc_com_pos_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("state_estimation/com_position", 10);

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

void SensorFusionNode::imuVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    m_imu_velocity = msg;
}

void SensorFusionNode::publishStateEstimation()
{
    march_shared_msgs::msg::StateEstimation state_estimation_msg;
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes(m_node_feet_names);

    // std::vector<geometry_msgs::msg::Pose> body_foot_poses = m_sensor_fusion->getFootPoses();
    std::vector<geometry_msgs::msg::Pose> body_foot_poses;
    try {
        geometry_msgs::msg::PoseStamped foot_pose;
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Get left foot positions w.r.t. backpack frame
        transform_stamped = m_tf_buffer->lookupTransform("backpack", "L_ground", tf2::TimePointZero);
        foot_pose.pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.pose.position.z = transform_stamped.transform.translation.z;
        foot_pose.pose.orientation.x = transform_stamped.transform.rotation.x;
        foot_pose.pose.orientation.y = transform_stamped.transform.rotation.y;
        foot_pose.pose.orientation.z = transform_stamped.transform.rotation.z;
        foot_pose.pose.orientation.w = transform_stamped.transform.rotation.w;
        body_foot_poses.push_back(foot_pose.pose);

        // Get right foot positions in body frame w.r.t. backpack frame
        transform_stamped = m_tf_buffer->lookupTransform("backpack", "R_ground", tf2::TimePointZero);
        foot_pose.pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.pose.position.z = transform_stamped.transform.translation.z;
        foot_pose.pose.orientation.x = transform_stamped.transform.rotation.x;
        foot_pose.pose.orientation.y = transform_stamped.transform.rotation.y;
        foot_pose.pose.orientation.z = transform_stamped.transform.rotation.z;
        foot_pose.pose.orientation.w = transform_stamped.transform.rotation.w;
        body_foot_poses.push_back(foot_pose.pose);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting foot positions in body frame: %s", e.what());
        return;
    }

    uint8_t stance_leg = m_sensor_fusion->updateStanceLeg(&body_foot_poses[0].position, &body_foot_poses[1].position);

    std::vector<geometry_msgs::msg::Pose> inertial_foot_positions;
    try {
        geometry_msgs::msg::Pose foot_pose;
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Get left foot position in world frame w.r.t. right ground frame.
        transform_stamped = m_tf_buffer->lookupTransform("world", "L_ground", tf2::TimePointZero);
        foot_pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.position.z = transform_stamped.transform.translation.z;
        foot_pose.orientation.x = transform_stamped.transform.rotation.x;
        foot_pose.orientation.y = transform_stamped.transform.rotation.y;
        foot_pose.orientation.z = transform_stamped.transform.rotation.z;
        foot_pose.orientation.w = transform_stamped.transform.rotation.w;
        inertial_foot_positions.push_back(foot_pose);

        // Get right foot position in world frame w.r.t. right ground frame
        transform_stamped = m_tf_buffer->lookupTransform("world", "R_ground", tf2::TimePointZero);
        foot_pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.position.z = transform_stamped.transform.translation.z;
        foot_pose.orientation.x = transform_stamped.transform.rotation.x;
        foot_pose.orientation.y = transform_stamped.transform.rotation.y;
        foot_pose.orientation.z = transform_stamped.transform.rotation.z;
        foot_pose.orientation.w = transform_stamped.transform.rotation.w;
        inertial_foot_positions.push_back(foot_pose);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting foot positions in world frame: %s", e.what());
        return;
    }

    std::vector<std::string> ankle_names = {"left_ankle", "right_ankle"};
    std::vector<RobotNode::SharedPtr> ankle_nodes = m_robot_description->findNodes(ankle_names);
    std::vector<geometry_msgs::msg::Pose> body_ankle_poses;
    for (const auto& ankle_node : ankle_nodes) {
        geometry_msgs::msg::Pose ankle_pose;
        Eigen::Vector3d ankle_position = ankle_node->getGlobalPosition(m_sensor_fusion->getJointPositions());
        Eigen::Quaterniond ankle_orientation(ankle_node->getGlobalRotation(m_sensor_fusion->getJointPositions()));
        ankle_pose.position.x = ankle_position.x();
        ankle_pose.position.y = ankle_position.y();
        ankle_pose.position.z = ankle_position.z();
        ankle_pose.orientation.x = ankle_orientation.x();
        ankle_pose.orientation.y = ankle_orientation.y();
        ankle_pose.orientation.z = ankle_orientation.z();
        ankle_pose.orientation.w = ankle_orientation.w();
        body_ankle_poses.push_back(ankle_pose);
    }

    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "backpack";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = *m_joint_state;
    // state_estimation_msg.imu = *m_sensor_fusion->getFilteredImuMsg();
    state_estimation_msg.imu = *m_imu;
    state_estimation_msg.body_ankle_pose = body_ankle_poses;
    state_estimation_msg.foot_pose = body_foot_poses;
    state_estimation_msg.inertial_foot_position = inertial_foot_positions;
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
    // Wait for transform otherwise return
    if (!m_tf_buffer->canTransform("R_ground", "world", tf2::TimePointZero) || !m_tf_buffer->canTransform("R_ground", "L_ground", tf2::TimePointZero)) {
        RCLCPP_WARN(this->get_logger(), "Cannot transform from world to R_ground");
        return;
    }

    // Get current time
    rclcpp::Time current_time = this->now();

    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes(m_node_feet_names);
    std::vector<geometry_msgs::msg::Pose> foot_poses = m_sensor_fusion->getFootPoses();
    uint8_t stance_leg = m_sensor_fusion->updateStanceLeg(&foot_poses[0].position, &foot_poses[1].position);

    std::vector<geometry_msgs::msg::Pose> inertial_foot_positions;
    try {
        geometry_msgs::msg::Pose foot_pose;
        
        // Get left foot position w.r.t. right ground frame.
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = m_tf_buffer->lookupTransform("R_ground", "L_ground", tf2::TimePointZero);
        foot_pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.position.z = 0;
        foot_pose.orientation.x = 0;
        foot_pose.orientation.y = 0;
        foot_pose.orientation.z = 0;
        foot_pose.orientation.w = 1;
        inertial_foot_positions.push_back(foot_pose);

        // Get right foot position w.r.t. right ground frame
        // transform_stamped = m_tf_buffer->lookupTransform("world", "R_ground", tf2::TimePointZero);
        // foot_pose.position.x = transform_stamped.transform.translation.x;
        // foot_pose.position.y = transform_stamped.transform.translation.y;
        // foot_pose.position.z = 0;
        // foot_pose.orientation.x = 0;
        // foot_pose.orientation.y = 0;
        // foot_pose.orientation.z = 0;
        // foot_pose.orientation.w = 1;
        // inertial_foot_positions.push_back(foot_pose);
        foot_pose.position.x = 0.0;
        foot_pose.position.y = 0.0;
        foot_pose.position.z = 0.0;
        foot_pose.orientation.x = 0.0;
        foot_pose.orientation.y = 0.0;
        foot_pose.orientation.z = 0.0;
        foot_pose.orientation.w = 1.0;
        inertial_foot_positions.push_back(foot_pose);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting foot positions: %s", e.what());
        return;
    }

    // Get transform stamped from world to R_ground
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = m_tf_buffer->lookupTransform("R_ground", "world", tf2::TimePointZero);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting transform stamped: %s", e.what());
        return;
    }
    Eigen::Quaterniond q(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);

    // Calculate COM and ZMP
    Eigen::Vector3d com_position = m_sensor_fusion->getCOM() + Eigen::Vector3d(m_imu_position->point.x, m_imu_position->point.y, m_imu_position->point.z);
    Eigen::Vector3d com_velocity = m_sensor_fusion->getCOMVelocity() + Eigen::Vector3d(m_imu_velocity->vector.x, m_imu_velocity->vector.y, m_imu_velocity->vector.z);

    const double gravity = 9.81;
    double zmp_x = com_position.x() + (com_velocity.x() / sqrt(gravity / com_position.z())) * com_position.y();
    double zmp_y = com_position.y() + (com_velocity.y() / sqrt(gravity / com_position.z())) * com_position.z();

    // Transform COM and ZMP to R_ground frame
    Eigen::Vector3d com_position_world(com_position.x(), com_position.y(), com_position.z());
    Eigen::Vector3d com_position_R_ground = q * com_position_world;
    com_position_R_ground.x() += transform_stamped.transform.translation.x;
    com_position_R_ground.y() += transform_stamped.transform.translation.y;
    com_position_R_ground.z() += transform_stamped.transform.translation.z;

    Eigen::Vector3d com_velocity_world(com_velocity.x(), com_velocity.y(), com_velocity.z());
    Eigen::Vector3d com_velocity_R_ground = q * com_velocity_world;
    
    Eigen::Vector3d zmp_world(zmp_x, zmp_y, 0);
    Eigen::Vector3d zmp_R_ground = q * zmp_world;
    zmp_R_ground.x() += transform_stamped.transform.translation.x;
    zmp_R_ground.y() += transform_stamped.transform.translation.y;
    zmp_R_ground.z() += transform_stamped.transform.translation.z;

    // Publish MPC estimation
    geometry_msgs::msg::PoseArray foot_positions_msg;
    foot_positions_msg.header.stamp = current_time;
    foot_positions_msg.header.frame_id = "R_ground";
    foot_positions_msg.poses = inertial_foot_positions;
    m_mpc_foot_positions_pub->publish(foot_positions_msg);

    march_shared_msgs::msg::CenterOfMass com_msg;
    com_msg.header.stamp = current_time;
    com_msg.header.frame_id = "R_ground";
    com_msg.position.header = com_msg.header;
    com_msg.position.point.x = com_position_R_ground.x();
    com_msg.position.point.y = com_position_R_ground.y();
    com_msg.position.point.z = com_position_R_ground.z();
    com_msg.velocity.x = com_velocity_R_ground.x();
    com_msg.velocity.y = com_velocity_R_ground.y();
    com_msg.velocity.z = com_velocity_R_ground.z();
    m_mpc_com_pub->publish(com_msg);

    geometry_msgs::msg::PointStamped com_pos_msg;
    com_pos_msg.header.stamp = current_time;
    com_pos_msg.header.frame_id = "R_ground";
    com_pos_msg.point.x = com_position_R_ground.x();
    com_pos_msg.point.y = com_position_R_ground.y();
    com_pos_msg.point.z = com_position_R_ground.z();
    m_mpc_com_pos_pub->publish(com_pos_msg);

    geometry_msgs::msg::PointStamped zmp_msg;
    zmp_msg.header.stamp = current_time;
    zmp_msg.header.frame_id = "R_ground";
    zmp_msg.point.x = zmp_R_ground.x();
    zmp_msg.point.y = zmp_R_ground.y();
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