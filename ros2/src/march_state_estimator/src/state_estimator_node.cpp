/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/state_estimator_node.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <unordered_map>

#include "omp.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "math.h"

using std::placeholders::_1;
using std::placeholders::_2;

SensorFusionNode::SensorFusionNode(): Node("state_estimator")
{
    // Determine if it is a simulation or real robot
    declare_parameter("simulation", true);
    m_is_simulation = get_parameter("simulation").as_bool();
    RCLCPP_INFO(this->get_logger(), "Simulation: %s", m_is_simulation ? "true" : "false");

    // First initialize sensor values to zero
    // Initialize joint states to zero
    std::vector<std::string> joint_names = { 
        "left_ankle_dpf", "left_ankle_ie", "left_hip_aa", "left_hip_fe", "left_knee",
        "right_ankle_dpf", "right_ankle_ie", "right_hip_aa", "right_hip_fe", "right_knee"
    };
    m_joint_state = nullptr;
    // m_joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    // m_joint_state->header.frame_id = "joint_link";
    // m_joint_state->header.stamp = this->now();
    // m_joint_state->name = joint_names;
    // m_joint_state->position = std::vector<double>(joint_names.size(), 0.0);
    // m_joint_state->velocity = std::vector<double>(joint_names.size(), 0.0);
    // m_joint_state->effort = std::vector<double>(joint_names.size(), 0.0);
    m_joint_state_last_update = this->now();

    // Initialize IMU to identity quaternion and gravity in z-axis
    m_imu = std::make_shared<sensor_msgs::msg::Imu>();
    m_imu->header.frame_id = "backpack";
    m_imu->header.stamp = this->now();
    m_imu->orientation.w = 1.0;
    m_imu->orientation.x = 0.0;
    m_imu->orientation.y = 0.0;
    m_imu->orientation.z = 0.0;
    m_imu->angular_velocity.x = 0.0;
    m_imu->angular_velocity.y = 0.0;
    m_imu->angular_velocity.z = 0.0;
    m_imu->linear_acceleration.x = 0.0;
    m_imu->linear_acceleration.y = 0.0;
    m_imu->linear_acceleration.z = -9.81;
    m_imu_last_update = this->now();

    // Set timeout values in seconds
    // TODO: Set these values in a parameter file
    m_joint_state_timeout = 1.0;
    m_imu_timeout = 5.0;

    declare_parameter("robot_definition", std::string());
    std::string yaml_filename = get_parameter("robot_definition").as_string();

    if (yaml_filename.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No robot description file has been provided.");
        return;
    }

    m_robot_description = std::make_shared<RobotDescription>(yaml_filename);

    declare_parameter("urdf_file_path", std::string());
    std::string urdf_file_path = get_parameter("urdf_file_path").as_string();

    if (urdf_file_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No URDF file path has been provided.");
        return;
    }

    declare_parameter("clock_period", 0.05);
    m_dt = get_parameter("clock_period").as_double();

    declare_parameter("left_stance_threshold", 400.0);
    declare_parameter("right_stance_threshold", 400.0);
    double left_stance_threshold = get_parameter("left_stance_threshold").as_double();
    double right_stance_threshold = get_parameter("right_stance_threshold").as_double();

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    m_sensors_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_sensors_subscription_options.callback_group = m_sensors_callback_group;

    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states/filtered", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::jointStateCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("lower_imu/filtered", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_clock_pub = this->create_publisher<std_msgs::msg::Header>("state_estimation/clock", 10);
    m_state_estimation_pub
        = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10);
    m_feet_height_pub 
        = this->create_publisher<march_shared_msgs::msg::FeetHeightStamped>("state_estimation/feet_height", 10);
    m_torque_left_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("state_estimation/ground_reaction_force/left", 10);
    m_torque_right_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("state_estimation/ground_reaction_force/right", 10);

    // Simulation ground truth information about position and velocity in world frame
    m_imu_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("lower_imu/position", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuPositionCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_imu_velocity_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("lower_imu/velocity", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuVelocityCallback, this, std::placeholders::_1), m_sensors_subscription_options);

    // M8's MPC
    m_mpc_foot_positions_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("est_foot_position", 10);
    m_mpc_com_pub = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 10);
    m_mpc_zmp_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 10);
    m_mpc_stance_foot_pub = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 10);

    m_mpc_com_pos_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("state_estimation/com_position", 10);
    
    // Initialize sensor fusion
    m_node_feet_names = { "L_sole", "R_sole" };
    m_sensor_fusion = std::make_unique<SensorFusion>(m_robot_description, urdf_file_path);
    m_sensor_fusion->configureJointNames(joint_names);
    m_sensor_fusion->configureStanceThresholds(left_stance_threshold, right_stance_threshold);
    RCLCPP_INFO(this->get_logger(), "State estimator clock period: %f s", m_dt);

    // Initialize timer
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(m_dt * 1000)), std::bind(&SensorFusionNode::timerCallback, this), m_sensors_callback_group);

    RCLCPP_INFO(this->get_logger(), "State Estimator Node initialized");
}

SensorFusionNode::~SensorFusionNode()
{
    RCLCPP_WARN(rclcpp::get_logger("march_state_estimator"), "StateEstimatorNode has been stopped.");
}

void SensorFusionNode::timerCallback()
{
    // Check if joint state is initialized properly
    if (m_joint_state == nullptr) {
        return;
    }

    // Comment this out for real robot
    if (m_is_simulation && (m_imu_position == nullptr)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No imu position data received yet");
        return;
    }

    // Check if joint state and imu data are received
    checkJointStateTimeout();
    checkImuTimeout();

    // Update and estimate current state
    m_sensor_fusion->updateImuState(m_imu);
    if (m_is_simulation) {
        m_sensor_fusion->updateDynamicsState();
        // m_sensor_fusion->updateKalmanFilter();
    }
    broadcastTransformToTf2();

    // publishFeetHeight();
    publishMPCEstimation();
    if (m_is_simulation) {
        publishGroundReactionForce();
    }
    
    publishStateEstimation();
    publishClock();
}

void SensorFusionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    m_joint_state = msg;
    m_sensor_fusion->updateJointState(m_joint_state);
    m_joint_state_last_update = this->now();
}

void SensorFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu = msg;
    m_imu_last_update = this->now();
}

void SensorFusionNode::imuPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_imu_position = msg;
}

void SensorFusionNode::imuVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    m_imu_velocity = msg;
}

void SensorFusionNode::publishClock()
{
    std_msgs::msg::Header clock_msg;
    clock_msg.stamp = this->now();
    m_clock_pub->publish(clock_msg);
}

void SensorFusionNode::publishStateEstimation()
{
    march_shared_msgs::msg::StateEstimation state_estimation_msg;
    rclcpp::Time current_time = current_time;

    std::vector<geometry_msgs::msg::Pose> body_sole_poses;
    try {
        geometry_msgs::msg::PoseStamped sole_pose;
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Get left foot positions w.r.t. backpack frame
        transform_stamped = m_tf_buffer->lookupTransform("backpack", "L_sole", tf2::TimePointZero);
        sole_pose.pose.position.x = transform_stamped.transform.translation.x;
        sole_pose.pose.position.y = transform_stamped.transform.translation.y;
        sole_pose.pose.position.z = transform_stamped.transform.translation.z;
        sole_pose.pose.orientation.x = transform_stamped.transform.rotation.x;
        sole_pose.pose.orientation.y = transform_stamped.transform.rotation.y;
        sole_pose.pose.orientation.z = transform_stamped.transform.rotation.z;
        sole_pose.pose.orientation.w = transform_stamped.transform.rotation.w;
        body_sole_poses.push_back(sole_pose.pose);

        // Get right foot positions in body frame w.r.t. backpack frame
        transform_stamped = m_tf_buffer->lookupTransform("backpack", "R_sole", tf2::TimePointZero);
        sole_pose.pose.position.x = transform_stamped.transform.translation.x;
        sole_pose.pose.position.y = transform_stamped.transform.translation.y;
        sole_pose.pose.position.z = transform_stamped.transform.translation.z;
        sole_pose.pose.orientation.x = transform_stamped.transform.rotation.x;
        sole_pose.pose.orientation.y = transform_stamped.transform.rotation.y;
        sole_pose.pose.orientation.z = transform_stamped.transform.rotation.z;
        sole_pose.pose.orientation.w = transform_stamped.transform.rotation.w;
        body_sole_poses.push_back(sole_pose.pose);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting foot positions in body frame: %s", e.what());
        return;
    }

    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "base_link";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = m_sensor_fusion->getEstimatedJointState();
    state_estimation_msg.joint_state.header.stamp = this->now();
    state_estimation_msg.joint_state.header.frame_id = m_joint_state->header.frame_id;

    if (m_is_simulation) {
        state_estimation_msg.dynamical_joint_state.header.stamp = current_time;
        state_estimation_msg.header.frame_id = "joint_link";
        state_estimation_msg.dynamical_joint_state.joint_name = m_joint_state->name;
        state_estimation_msg.dynamical_joint_state.joint_acceleration = m_sensor_fusion->getJointAcceleration(m_joint_state->name);
        state_estimation_msg.dynamical_joint_state.effort_dynamical = m_sensor_fusion->getJointDynamicalTorques(m_joint_state->name);
        state_estimation_msg.dynamical_joint_state.effort_external = m_sensor_fusion->getJointExternalTorques(m_joint_state->name);
    }

    // state_estimation_msg.imu = *m_sensor_fusion->getFilteredImuMsg();
    state_estimation_msg.imu = *m_imu;
    state_estimation_msg.body_ankle_pose = getCurrentPoseArray("backpack", {"L_ankle", "R_ankle"});
    state_estimation_msg.body_foot_pose = getCurrentPoseArray("backpack", {"L_sole", "R_sole"});
    state_estimation_msg.current_stance_leg = m_sensor_fusion->getCurrentStanceLeg();
    state_estimation_msg.next_stance_leg = m_sensor_fusion->getNextStanceLeg(
        state_estimation_msg.body_ankle_pose[LEFT_FOOT_ID].position.x,
        state_estimation_msg.body_ankle_pose[RIGHT_FOOT_ID].position.x);
    
    if (m_is_simulation) {
        state_estimation_msg.world_foot_pose = getCurrentPoseArray("world", {"L_sole", "R_sole"});
    }
    
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
    if (!m_tf_buffer->canTransform("R_sole", "world", tf2::TimePointZero) || !m_tf_buffer->canTransform("R_sole", "L_sole", tf2::TimePointZero)) {
        RCLCPP_WARN(this->get_logger(), "Cannot transform from world to R_sole");
        return;
    }

    // Get current time
    rclcpp::Time current_time = this->now();

    // uint8_t stance_leg = m_sensor_fusion->getCurrentStanceLeg();
    std::vector<geometry_msgs::msg::Pose> foot_positions = getCurrentPoseArray("backpack", {"L_heel", "R_heel"});
    uint8_t stance_leg = m_sensor_fusion->getNextStanceLeg(foot_positions[0].position.x, foot_positions[1].position.x);

    // Get transform stamped from world to R_sole
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = m_tf_buffer->lookupTransform("R_sole", "world", tf2::TimePointZero);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting transform stamped: %s", e.what());
        return;
    }
    Eigen::Quaterniond q(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);
    q.normalize();

    // Calculate COM and ZMP
    Eigen::Vector3d com_position = m_sensor_fusion->getCOM() + Eigen::Vector3d(m_imu_position->point.x, m_imu_position->point.y, m_imu_position->point.z);
    Eigen::Vector3d com_velocity = m_sensor_fusion->getCOMVelocity() + Eigen::Vector3d(m_imu_velocity->vector.x, m_imu_velocity->vector.y, m_imu_velocity->vector.z);

    const double gravity = 9.81;
    double zmp_x = com_position.x() + (com_velocity.x() / sqrt(gravity / com_position.z())) * com_position.y();
    double zmp_y = com_position.y() + (com_velocity.y() / sqrt(gravity / com_position.z())) * com_position.z();

    // Transform COM and ZMP to R_sole frame
    Eigen::Vector3d com_position_world(com_position.x(), com_position.y(), com_position.z());
    Eigen::Vector3d com_position_R_sole = q * com_position_world;
    com_position_R_sole.x() += transform_stamped.transform.translation.x;
    com_position_R_sole.y() += transform_stamped.transform.translation.y;
    com_position_R_sole.z() += transform_stamped.transform.translation.z;

    Eigen::Vector3d com_velocity_world(com_velocity.x(), com_velocity.y(), com_velocity.z());
    Eigen::Vector3d com_velocity_R_sole = q * com_velocity_world;
    
    Eigen::Vector3d zmp_world(zmp_x, zmp_y, 0);
    Eigen::Vector3d zmp_R_sole = q * zmp_world;
    zmp_R_sole.x() += transform_stamped.transform.translation.x;
    zmp_R_sole.y() += transform_stamped.transform.translation.y;
    zmp_R_sole.z() += transform_stamped.transform.translation.z;

    // Publish MPC estimation
    geometry_msgs::msg::PoseArray foot_positions_msg;
    foot_positions_msg.header.stamp = current_time;
    foot_positions_msg.header.frame_id = "R_sole";
    foot_positions_msg.poses = getCurrentPoseArray("R_sole", {"L_sole", "R_sole"});
    m_mpc_foot_positions_pub->publish(foot_positions_msg);

    march_shared_msgs::msg::CenterOfMass com_msg;
    com_msg.header.stamp = current_time;
    com_msg.header.frame_id = "R_sole";
    com_msg.position.header = com_msg.header;
    com_msg.position.point.x = com_position_R_sole.x();
    com_msg.position.point.y = com_position_R_sole.y();
    com_msg.position.point.z = com_position_R_sole.z();
    com_msg.velocity.x = com_velocity_R_sole.x();
    com_msg.velocity.y = com_velocity_R_sole.y();
    com_msg.velocity.z = com_velocity_R_sole.z();
    m_mpc_com_pub->publish(com_msg);

    geometry_msgs::msg::PointStamped com_pos_msg;
    com_pos_msg.header.stamp = current_time;
    com_pos_msg.header.frame_id = "R_sole";
    com_pos_msg.point.x = com_position_R_sole.x();
    com_pos_msg.point.y = com_position_R_sole.y();
    com_pos_msg.point.z = com_position_R_sole.z();
    m_mpc_com_pos_pub->publish(com_pos_msg);

    geometry_msgs::msg::PointStamped zmp_msg;
    zmp_msg.header.stamp = current_time;
    zmp_msg.header.frame_id = "R_sole";
    zmp_msg.point.x = zmp_R_sole.x();
    zmp_msg.point.y = zmp_R_sole.y();
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

void SensorFusionNode::publishGroundReactionForce()
{
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>  torque_pubs = { m_torque_left_pub, m_torque_right_pub };
    std::vector<Eigen::Vector3d> foot_forces = { m_sensor_fusion->getLeftFootForce(), m_sensor_fusion->getRightFootForce() };
    rclcpp::Time current_time = this->now();

    for (size_t i = 0; i < torque_pubs.size(); i++) {
        geometry_msgs::msg::Vector3Stamped torque_msg;
        torque_msg.header.stamp = current_time;
        torque_msg.header.frame_id = "world";
        torque_msg.vector.x = foot_forces[i].x();
        torque_msg.vector.y = foot_forces[i].y();
        torque_msg.vector.z = foot_forces[i].z();
        torque_pubs[i]->publish(torque_msg);
    }
}

void SensorFusionNode::broadcastTransformToTf2()
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "base_link";
    // transform_stamped.transform = m_sensor_fusion->getRobotTransform();
    
    if (m_is_simulation) {
        transform_stamped.transform.translation.x = m_imu_position->point.x;
        transform_stamped.transform.translation.y = m_imu_position->point.y;
        transform_stamped.transform.translation.z = m_imu_position->point.z;
    } else {
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
    }
    transform_stamped.transform.rotation.x = m_imu->orientation.x;
    transform_stamped.transform.rotation.y = m_imu->orientation.y;
    transform_stamped.transform.rotation.z = m_imu->orientation.z;
    transform_stamped.transform.rotation.w = m_imu->orientation.w;

    m_tf_broadcaster->sendTransform(transform_stamped);
}

void SensorFusionNode::checkJointStateTimeout()
{
    if ((this->now() - m_joint_state_last_update) > rclcpp::Duration::from_seconds(m_joint_state_timeout)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No joint state data received yet");
    }
}

void SensorFusionNode::checkImuTimeout()
{
    if ((this->now() - m_imu_last_update) > rclcpp::Duration::from_seconds(m_imu_timeout)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No imu data received yet");
    }
}

geometry_msgs::msg::TransformStamped SensorFusionNode::getCurrentTransform(const std::string& parent_frame, const std::string& child_frame)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = m_tf_buffer->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting transform stamped: %s", e.what());
    }
    return transform_stamped;
}

geometry_msgs::msg::Pose SensorFusionNode::getCurrentPose(const std::string& parent_frame, const std::string& child_frame)
{
    geometry_msgs::msg::Pose pose;
    try {
        geometry_msgs::msg::TransformStamped transform_stamped = getCurrentTransform(parent_frame, child_frame);
        pose.position.x = transform_stamped.transform.translation.x;
        pose.position.y = transform_stamped.transform.translation.y;
        pose.position.z = transform_stamped.transform.translation.z;
        pose.orientation.x = transform_stamped.transform.rotation.x;
        pose.orientation.y = transform_stamped.transform.rotation.y;
        pose.orientation.z = transform_stamped.transform.rotation.z;
        pose.orientation.w = transform_stamped.transform.rotation.w;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting pose: %s", e.what());
    }
    return pose;
}

std::vector<geometry_msgs::msg::Pose> SensorFusionNode::getCurrentPoseArray(const std::string& parent_frame, const std::vector<std::string>& child_frames)
{
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const std::string& child_frame : child_frames) {
        poses.push_back(getCurrentPose(parent_frame, child_frame));
    }
    return poses;
}

int main(int argc, char** argv)
{
    Eigen::initParallel();
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_sensor_fusion = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node_sensor_fusion);
    rclcpp::shutdown();
    return 0;
}