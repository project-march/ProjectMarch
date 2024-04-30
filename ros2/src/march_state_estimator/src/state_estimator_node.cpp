/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/state_estimator_node.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

SensorFusionNode::SensorFusionNode()
    : Node("state_estimator_node")
{
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

    declare_parameter("timestep_in_ms", 50);
    int dt = get_parameter("timestep_in_ms").as_int();

    declare_parameter("left_stance_threshold", 400.0);
    declare_parameter("right_stance_threshold", 400.0);
    double left_stance_threshold = get_parameter("left_stance_threshold").as_double();
    double right_stance_threshold = get_parameter("right_stance_threshold").as_double();

    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    m_sensors_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_sensors_subscription_options.callback_group = m_sensors_callback_group;

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(dt), std::bind(&SensorFusionNode::timerCallback, this), m_sensors_callback_group);
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states/filtered", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::jointStateCallback, this, std::placeholders::_1),
        m_sensors_subscription_options);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("lower_imu/filtered", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_imu_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("lower_imu/position", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuPositionCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_imu_velocity_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("lower_imu/velocity", rclcpp::SensorDataQoS(),
        std::bind(&SensorFusionNode::imuVelocityCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_state_estimation_pub
        = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10);
    m_feet_height_pub 
        = this->create_publisher<march_shared_msgs::msg::FeetHeightStamped>("state_estimation/feet_height", 10);
    m_torque_left_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("state_estimation/torque/left", 10);
    m_torque_right_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("state_estimation/torque/right", 10);

    // M8's MPC
    m_mpc_foot_positions_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("est_foot_position", 10);
    m_mpc_com_pub = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 10);
    m_mpc_zmp_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 10);
    m_mpc_stance_foot_pub = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 10);

    m_mpc_com_pos_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("state_estimation/com_position", 10);

    std::vector<std::string> joint_names = { 
        "left_ankle_dpf", "left_ankle_ie", "left_hip_aa", "left_hip_fe", "left_knee",
        "right_ankle_dpf", "right_ankle_ie", "right_hip_aa", "right_hip_fe", "right_knee"
    };
    m_sensor_fusion = std::make_unique<SensorFusion>(m_robot_description, urdf_file_path);
    m_sensor_fusion->configureJointNames(joint_names);
    m_sensor_fusion->configureStanceThresholds(left_stance_threshold, right_stance_threshold);
    m_dt = static_cast<double>(dt) / 1000.0;
    m_joint_state = nullptr;
    m_imu = nullptr;
    m_node_feet_names = { "L_sole", "R_sole" };

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

    // Update and estimate current state
    m_sensor_fusion->updateJointState(m_joint_state);
    m_sensor_fusion->updateImuState(m_imu);
    m_sensor_fusion->updateDynamicsState();
    // m_sensor_fusion->updateKalmanFilter();

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "base_link";
    // transform_stamped.transform = m_sensor_fusion->getRobotTransform();
    transform_stamped.transform.translation.x = m_imu_position->point.x;
    transform_stamped.transform.translation.y = m_imu_position->point.y;
    transform_stamped.transform.translation.z = m_imu_position->point.z;
    transform_stamped.transform.rotation.x = m_imu->orientation.x;
    transform_stamped.transform.rotation.y = m_imu->orientation.y;
    transform_stamped.transform.rotation.z = m_imu->orientation.z;
    transform_stamped.transform.rotation.w = m_imu->orientation.w;
    m_tf_broadcaster->sendTransform(transform_stamped);

    // publishFeetHeight();
    // publishMPCEstimation();
    publishTorqueEstimation();
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
        transform_stamped = m_tf_buffer->lookupTransform("backpack", "L_heel", tf2::TimePointZero);
        foot_pose.pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.pose.position.z = transform_stamped.transform.translation.z;
        foot_pose.pose.orientation.x = transform_stamped.transform.rotation.x;
        foot_pose.pose.orientation.y = transform_stamped.transform.rotation.y;
        foot_pose.pose.orientation.z = transform_stamped.transform.rotation.z;
        foot_pose.pose.orientation.w = transform_stamped.transform.rotation.w;
        body_foot_poses.push_back(foot_pose.pose);

        // Get right foot positions in body frame w.r.t. backpack frame
        transform_stamped = m_tf_buffer->lookupTransform("backpack", "R_heel", tf2::TimePointZero);
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

    std::vector<geometry_msgs::msg::Pose> inertial_foot_positions;
    try {
        geometry_msgs::msg::Pose foot_pose;
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Get left foot position in world frame w.r.t. right ground frame.
        transform_stamped = m_tf_buffer->lookupTransform("world", "L_heel", tf2::TimePointZero);
        foot_pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.position.z = transform_stamped.transform.translation.z;
        foot_pose.orientation.x = transform_stamped.transform.rotation.x;
        foot_pose.orientation.y = transform_stamped.transform.rotation.y;
        foot_pose.orientation.z = transform_stamped.transform.rotation.z;
        foot_pose.orientation.w = transform_stamped.transform.rotation.w;
        inertial_foot_positions.push_back(foot_pose);

        // Get right foot position in world frame w.r.t. right ground frame
        transform_stamped = m_tf_buffer->lookupTransform("world", "R_heel", tf2::TimePointZero);

        foot_pose.orientation.y = transform_stamped.transform.rotation.y;
        foot_pose.orientation.z = transform_stamped.transform.rotation.z;
        foot_pose.orientation.w = transform_stamped.transform.rotation.w;
        inertial_foot_positions.push_back(foot_pose);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting foot positions in world frame: %s", e.what());
        return;
    }

    std::vector<std::string> ankle_names = {"L_ankle", "R_ankle"};
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
    state_estimation_msg.header.frame_id = "joint_link";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = *m_joint_state;
    state_estimation_msg.dynamical_joint_state.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "joint_link";
    state_estimation_msg.dynamical_joint_state.joint_name = m_joint_state->name;
    state_estimation_msg.dynamical_joint_state.joint_acceleration = m_sensor_fusion->getJointAcceleration(m_joint_state->name);
    state_estimation_msg.dynamical_joint_state.effort_dynamical = m_sensor_fusion->getJointDynamicalTorques(m_joint_state->name);
    state_estimation_msg.dynamical_joint_state.effort_external = m_sensor_fusion->getJointExternalTorques(m_joint_state->name);

    // state_estimation_msg.imu = *m_sensor_fusion->getFilteredImuMsg();
    state_estimation_msg.imu = *m_imu;
    state_estimation_msg.body_ankle_pose = body_ankle_poses;
    state_estimation_msg.body_sole_pose = body_sole_poses;
    state_estimation_msg.foot_pose = body_foot_poses;
    state_estimation_msg.inertial_foot_position = inertial_foot_positions;
    state_estimation_msg.current_stance_leg = m_sensor_fusion->getCurrentStanceLeg();
    state_estimation_msg.next_stance_leg = m_sensor_fusion->getNextStanceLeg(body_foot_poses[LEFT_FOOT_ID].position.x, body_foot_poses[RIGHT_FOOT_ID].position.x);
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
    if (!m_tf_buffer->canTransform("R_heel", "world", tf2::TimePointZero) || !m_tf_buffer->canTransform("R_heel", "L_heel", tf2::TimePointZero)) {
        RCLCPP_WARN(this->get_logger(), "Cannot transform from world to R_heel");
        return;
    }

    // Get current time
    rclcpp::Time current_time = this->now();

    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes(m_node_feet_names);
    std::vector<geometry_msgs::msg::Pose> foot_poses = m_sensor_fusion->getFootPoses();
    uint8_t stance_leg = m_sensor_fusion->getCurrentStanceLeg();

    std::vector<geometry_msgs::msg::Pose> inertial_foot_positions;
    try {
        geometry_msgs::msg::Pose foot_pose;
        
        // Get left foot position w.r.t. right ground frame.
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = m_tf_buffer->lookupTransform("R_heel", "L_heel", tf2::TimePointZero);
        foot_pose.position.x = transform_stamped.transform.translation.x;
        foot_pose.position.y = transform_stamped.transform.translation.y;
        foot_pose.position.z = 0;
        foot_pose.orientation.x = 0;
        foot_pose.orientation.y = 0;
        foot_pose.orientation.z = 0;
        foot_pose.orientation.w = 1;
        inertial_foot_positions.push_back(foot_pose);

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

    // Get transform stamped from world to R_heel
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = m_tf_buffer->lookupTransform("R_heel", "world", tf2::TimePointZero);
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

    // Transform COM and ZMP to R_heel frame
    Eigen::Vector3d com_position_world(com_position.x(), com_position.y(), com_position.z());
    Eigen::Vector3d com_position_R_heel = q * com_position_world;
    com_position_R_heel.x() += transform_stamped.transform.translation.x;
    com_position_R_heel.y() += transform_stamped.transform.translation.y;
    com_position_R_heel.z() += transform_stamped.transform.translation.z;

    Eigen::Vector3d com_velocity_world(com_velocity.x(), com_velocity.y(), com_velocity.z());
    Eigen::Vector3d com_velocity_R_heel = q * com_velocity_world;
    
    Eigen::Vector3d zmp_world(zmp_x, zmp_y, 0);
    Eigen::Vector3d zmp_R_heel = q * zmp_world;
    zmp_R_heel.x() += transform_stamped.transform.translation.x;
    zmp_R_heel.y() += transform_stamped.transform.translation.y;
    zmp_R_heel.z() += transform_stamped.transform.translation.z;

    // Publish MPC estimation
    geometry_msgs::msg::PoseArray foot_positions_msg;
    foot_positions_msg.header.stamp = current_time;
    foot_positions_msg.header.frame_id = "R_heel";
    foot_positions_msg.poses = inertial_foot_positions;
    m_mpc_foot_positions_pub->publish(foot_positions_msg);

    march_shared_msgs::msg::CenterOfMass com_msg;
    com_msg.header.stamp = current_time;
    com_msg.header.frame_id = "R_heel";
    com_msg.position.header = com_msg.header;
    com_msg.position.point.x = com_position_R_heel.x();
    com_msg.position.point.y = com_position_R_heel.y();
    com_msg.position.point.z = com_position_R_heel.z();
    com_msg.velocity.x = com_velocity_R_heel.x();
    com_msg.velocity.y = com_velocity_R_heel.y();
    com_msg.velocity.z = com_velocity_R_heel.z();
    m_mpc_com_pub->publish(com_msg);

    geometry_msgs::msg::PointStamped com_pos_msg;
    com_pos_msg.header.stamp = current_time;
    com_pos_msg.header.frame_id = "R_heel";
    com_pos_msg.point.x = com_position_R_heel.x();
    com_pos_msg.point.y = com_position_R_heel.y();
    com_pos_msg.point.z = com_position_R_heel.z();
    m_mpc_com_pos_pub->publish(com_pos_msg);

    geometry_msgs::msg::PointStamped zmp_msg;
    zmp_msg.header.stamp = current_time;
    zmp_msg.header.frame_id = "R_heel";
    zmp_msg.point.x = zmp_R_heel.x();
    zmp_msg.point.y = zmp_R_heel.y();
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

void SensorFusionNode::publishTorqueEstimation()
{
    Eigen::Vector3d m_left_foot_force = m_sensor_fusion->getLeftFootForce();
    Eigen::Vector3d m_right_foot_force = m_sensor_fusion->getRightFootForce();

    geometry_msgs::msg::Vector3Stamped torque_left_msg;
    torque_left_msg.header.stamp = this->now();
    torque_left_msg.header.frame_id = "world";
    torque_left_msg.vector.x = m_left_foot_force.x();
    torque_left_msg.vector.y = m_left_foot_force.y();
    torque_left_msg.vector.z = m_left_foot_force.z();
    m_torque_left_pub->publish(torque_left_msg);

    geometry_msgs::msg::Vector3Stamped torque_right_msg;
    torque_right_msg.header.stamp = this->now();
    torque_right_msg.header.frame_id = "world";
    torque_right_msg.vector.x = m_right_foot_force.x();
    torque_right_msg.vector.y = m_right_foot_force.y();
    torque_right_msg.vector.z = m_right_foot_force.z();
    m_torque_right_pub->publish(torque_right_msg);
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