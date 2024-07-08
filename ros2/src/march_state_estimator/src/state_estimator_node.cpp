/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

// #define DEBUG

#include "march_state_estimator/state_estimator_node.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <unordered_map>

#include "omp.h"
#include "urdf/model.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "math.h"

using std::placeholders::_1;
using std::placeholders::_2;

StateEstimatorNode::StateEstimatorNode(): LifecycleNode("state_estimator")
{
    RCLCPP_INFO(rclcpp::get_logger("march_state_estimator"), "StateEstimatorNode is initializing...");
    declareParameters();
    RCLCPP_INFO(rclcpp::get_logger("march_state_estimator"), "StateEstimatorNode has been created.\nOn standby for configuration.");

    // Delay the initialization of the node until the lifecycle manager activates it
    rclcpp::Time start_time = this->now();
    int timeout = 5; // s
    while (this->now() - start_time < rclcpp::Duration(std::chrono::seconds(timeout)) && rclcpp::ok()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting to stabilize...");
    }
    this->on_configure(this->get_current_state());
    this->on_activate(this->get_current_state());
}

StateEstimatorNode::~StateEstimatorNode()
{
    RCLCPP_WARN(rclcpp::get_logger("march_state_estimator"), "StateEstimatorNode has been destroyed.");
}


/*******************************************************************************
 * Lifecycle node callbacks
 *******************************************************************************/

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateEstimatorNode::on_configure(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "State Estimator Node is configuring...");

    // Determine if it is a simulation or real robot
    m_is_simulation = get_parameter("simulation").as_bool();
    RCLCPP_INFO(this->get_logger(), "Simulation: %s", m_is_simulation ? "true" : "false");

    // Initialize the sensor messages
    if (!configureJointStateMsg()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize joint state message.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    if (!configureImuMsg()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU message.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Initialize the ROS2 communication
    configureSubscriptions();
    configurePublishers();
    configureStateEstimationTimer();
    configureTF2();

    // Initialize the state estimator and sensor fusion
    if (!configureStateEstimator()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize state estimator.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    if (!configureSensorFusion()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor fusion.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "State Estimator Node is fully configured.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateEstimatorNode::on_activate(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "State Estimator Node is activating...");

    // Activate lifecycle publishers
    m_clock_pub->on_activate();
    m_state_estimation_pub->on_activate();
    m_feet_height_pub->on_activate();
    m_torque_left_pub->on_activate();
    m_torque_right_pub->on_activate();
    m_mpc_foot_positions_pub->on_activate();
    m_mpc_com_pub->on_activate();
    m_mpc_zmp_pub->on_activate();
    m_mpc_stance_foot_pub->on_activate();
    m_mpc_com_pos_pub->on_activate();

    // Start the timer
    m_timer->reset();

    // Initialize the last update time
    updateJointStateTimeout();
    updateImuTimeout();

    RCLCPP_INFO(this->get_logger(), "State Estimator Node is active");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateEstimatorNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "State Estimator Node is deactivating...");

    m_clock_pub->on_deactivate();
    m_state_estimation_pub->on_deactivate();
    m_feet_height_pub->on_deactivate();
    m_torque_left_pub->on_deactivate();
    m_torque_right_pub->on_deactivate();
    m_mpc_foot_positions_pub->on_deactivate();
    m_mpc_com_pub->on_deactivate();
    m_mpc_zmp_pub->on_deactivate();
    m_mpc_stance_foot_pub->on_deactivate();
    m_mpc_com_pos_pub->on_deactivate();

    m_timer->cancel();
    RCLCPP_INFO(this->get_logger(), "State Estimator Node is inactive");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateEstimatorNode::on_cleanup(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "State Estimator Node is cleaning up...");

    m_state_estimation_pub.reset();
    m_feet_height_pub.reset();
    m_torque_left_pub.reset();
    m_torque_right_pub.reset();
    m_mpc_foot_positions_pub.reset();
    m_mpc_com_pub.reset();
    m_mpc_zmp_pub.reset();
    m_mpc_stance_foot_pub.reset();
    m_mpc_com_pos_pub.reset();

    m_timer.reset();
    m_joint_state_sub.reset();
    m_imu_sub.reset();
    m_imu_position_sub.reset();
    m_imu_velocity_sub.reset();

    m_robot_description.reset();
    m_state_estimator.reset();
    m_sensor_fusion.reset();

    m_tf_listener.reset();
    m_tf_buffer.reset();
    m_tf_broadcaster.reset();

    RCLCPP_INFO(this->get_logger(), "State Estimator Node has been cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateEstimatorNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
    (void) state;
    RCLCPP_INFO(this->get_logger(), "State Estimator Node is shutting down...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/*******************************************************************************
 * Subscription callbacks
 *******************************************************************************/

void StateEstimatorNode::timerCallback()
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
    m_state_estimator->updateImuState(m_imu);
    if (m_is_simulation) {
        m_state_estimator->updateDynamicsState();

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Updating measurement data...");
        #endif

        // Update joint position and stance leg
        m_sensor_fusion->setJointPosition(m_joint_state->name, m_joint_state->position);
        m_sensor_fusion->updateStanceLeg(m_state_estimator->getCurrentStanceLeg());
        
        // Update observation
        EKFObservation ekf_observation;
        ekf_observation.imu_acceleration = Eigen::Vector3d(m_imu->linear_acceleration.x, m_imu->linear_acceleration.y, m_imu->linear_acceleration.z);
        ekf_observation.imu_angular_velocity.noalias() = Eigen::Vector3d(m_imu->angular_velocity.x, m_imu->angular_velocity.y, m_imu->angular_velocity.z);
        std::vector<geometry_msgs::msg::Pose> body_foot_poses = getCurrentPoseArray("backpack", {"L_sole", "R_sole"});
        ekf_observation.left_foot_position = Eigen::Vector3d(
            body_foot_poses[LEFT_FOOT_ID].position.x, 
            body_foot_poses[LEFT_FOOT_ID].position.y, 
            body_foot_poses[LEFT_FOOT_ID].position.z);
        ekf_observation.right_foot_position = Eigen::Vector3d(
            body_foot_poses[RIGHT_FOOT_ID].position.x, 
            body_foot_poses[RIGHT_FOOT_ID].position.y, 
            body_foot_poses[RIGHT_FOOT_ID].position.z);
        ekf_observation.left_foot_slippage = Eigen::Quaterniond(
            body_foot_poses[LEFT_FOOT_ID].orientation.w, 
            body_foot_poses[LEFT_FOOT_ID].orientation.x, 
            body_foot_poses[LEFT_FOOT_ID].orientation.y, 
            body_foot_poses[LEFT_FOOT_ID].orientation.z).inverse();
        ekf_observation.right_foot_slippage = Eigen::Quaterniond(
            body_foot_poses[RIGHT_FOOT_ID].orientation.w, 
            body_foot_poses[RIGHT_FOOT_ID].orientation.x, 
            body_foot_poses[RIGHT_FOOT_ID].orientation.y, 
            body_foot_poses[RIGHT_FOOT_ID].orientation.z).inverse();
        m_sensor_fusion->setObservation(ekf_observation);

        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Estimating state...");
        #endif
        m_sensor_fusion->estimateState();
    }
    broadcastTransformToTf2();

    // publishFeetHeight();
    // publishMPCEstimation();
    if (m_is_simulation) {
        publishGroundReactionForce();
    }
    
    publishStateEstimation();
    publishClock();
}

void StateEstimatorNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    m_joint_state = msg;
    m_state_estimator->updateJointState(m_joint_state);
    m_joint_state_last_update = this->now();
}

void StateEstimatorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu = msg;
    m_imu_last_update = this->now();
}

void StateEstimatorNode::imuPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_imu_position = msg;
}

void StateEstimatorNode::imuVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    m_imu_velocity = msg;
}

void StateEstimatorNode::noiseParametersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 3) {
        RCLCPP_ERROR(this->get_logger(), "Invalid noise parameters size: %d", msg->data.size());
        return;
    }

    std::vector<double> foot_position_noise = { msg->data[0], msg->data[0], msg->data[0] };
    std::vector<double> foot_slippage_noise = { msg->data[1], msg->data[1], msg->data[1] };
    std::vector<double> joint_position_noise = { 
        msg->data[2], msg->data[2], msg->data[2], msg->data[2], msg->data[2],
        msg->data[2], msg->data[2], msg->data[2], msg->data[2], msg->data[2]
    };

    m_sensor_fusion->setObservationNoiseCovarianceMatrix(foot_position_noise, foot_slippage_noise, joint_position_noise);

    RCLCPP_WARN(this->get_logger(), "Observation noise parameters have been updated. Foot position: %f, Foot slippage: %f, Joint position: %f",
        foot_position_noise[0], foot_slippage_noise[0], joint_position_noise[0]);
}


/*******************************************************************************
 * Publisher functions
 *******************************************************************************/

void StateEstimatorNode::publishClock()
{
    std_msgs::msg::Header clock_msg;
    clock_msg.stamp = this->now();
    m_clock_pub->publish(clock_msg);
}

void StateEstimatorNode::publishStateEstimation()
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
        m_sensor_fusion_valid = false;
        return;
    }

    state_estimation_msg.header.stamp = this->now();
    state_estimation_msg.header.frame_id = "base_link";
    state_estimation_msg.step_time = m_dt;
    state_estimation_msg.joint_state = m_state_estimator->getEstimatedJointState();
    state_estimation_msg.joint_state.header.stamp = this->now();
    state_estimation_msg.joint_state.header.frame_id = m_joint_state->header.frame_id;

    if (m_is_simulation) {
        state_estimation_msg.dynamical_joint_state.header.stamp = current_time;
        state_estimation_msg.header.frame_id = "joint_link";
        state_estimation_msg.dynamical_joint_state.joint_name = m_joint_state->name;
        state_estimation_msg.dynamical_joint_state.joint_acceleration = m_state_estimator->getJointAcceleration(m_joint_state->name);
        state_estimation_msg.dynamical_joint_state.effort_dynamical = m_state_estimator->getJointDynamicalTorques(m_joint_state->name);
        state_estimation_msg.dynamical_joint_state.effort_external = m_state_estimator->getJointExternalTorques(m_joint_state->name);
    }

    // state_estimation_msg.imu = *m_state_estimator->getFilteredImuMsg();
    state_estimation_msg.imu = *m_imu;
    state_estimation_msg.body_ankle_pose = getCurrentPoseArray("backpack", {"L_ankle", "R_ankle"});
    state_estimation_msg.body_foot_pose = getCurrentPoseArray("backpack", {"L_sole", "R_sole"});
    state_estimation_msg.current_stance_leg = m_state_estimator->getCurrentStanceLeg();
    state_estimation_msg.next_stance_leg = m_state_estimator->getNextStanceLeg(
        state_estimation_msg.body_ankle_pose[LEFT_FOOT_ID].position.x,
        state_estimation_msg.body_ankle_pose[RIGHT_FOOT_ID].position.x);
    
    if (m_is_simulation) {
        state_estimation_msg.world_foot_pose = getCurrentPoseArray("world", {"L_sole", "R_sole"});
    }

    // Variables for optimization of Kalman Filter tuning
    state_estimation_msg.performance_cost = m_sensor_fusion->getPerformanceCost();
    state_estimation_msg.sensor_fusion_valid = m_sensor_fusion_valid;
    
    m_state_estimation_pub->publish(state_estimation_msg);

    // Reset variables
    m_sensor_fusion_valid = true;
}

void StateEstimatorNode::publishFeetHeight()
{
    march_shared_msgs::msg::FeetHeightStamped feet_height_msg;
    feet_height_msg.header.stamp = this->now();
    feet_height_msg.header.frame_id = "world";
    feet_height_msg.heights = m_state_estimator->getFootContactHeight();
    m_feet_height_pub->publish(feet_height_msg);
}

void StateEstimatorNode::publishMPCEstimation()
{
    // Wait for transform otherwise return
    if (!m_tf_buffer->canTransform("R_heel", "world", tf2::TimePointZero) || !m_tf_buffer->canTransform("R_heel", "L_heel", tf2::TimePointZero)) {
        RCLCPP_WARN(this->get_logger(), "Cannot transform from world to R_heel");
        return;
    }

    // Get current time
    rclcpp::Time current_time = this->now();

    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes(m_node_feet_names);
    std::vector<geometry_msgs::msg::Pose> foot_poses = m_state_estimator->getFootPoses();
    uint8_t stance_leg = m_state_estimator->getCurrentStanceLeg();

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
        m_sensor_fusion_valid = false;
        return;
    }

    // Get transform stamped from world to R_heel
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = m_tf_buffer->lookupTransform("R_heel", "world", tf2::TimePointZero);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting transform stamped: %s", e.what());
        m_sensor_fusion_valid = false;
        return;
    }
    Eigen::Quaterniond q(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);

    // Calculate COM and ZMP
    Eigen::Vector3d com_position = m_state_estimator->getCOM() + Eigen::Vector3d(m_imu_position->point.x, m_imu_position->point.y, m_imu_position->point.z);
    Eigen::Vector3d com_velocity = m_state_estimator->getCOMVelocity() + Eigen::Vector3d(m_imu_velocity->vector.x, m_imu_velocity->vector.y, m_imu_velocity->vector.z);

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

void StateEstimatorNode::publishGroundReactionForce()
{
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>  torque_pubs = { m_torque_left_pub, m_torque_right_pub };
    std::vector<Eigen::Vector3d> foot_forces = { m_state_estimator->getLeftFootForce(), m_state_estimator->getRightFootForce() };
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

void StateEstimatorNode::broadcastTransformToTf2()
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "base_link";
    // transform_stamped.transform = m_state_estimator->getRobotTransform();
    
    // if (m_is_simulation) {
    //     transform_stamped.transform.translation.x = m_imu_position->point.x;
    //     transform_stamped.transform.translation.y = m_imu_position->point.y;
    //     transform_stamped.transform.translation.z = m_imu_position->point.z;
    // } else {
    //     transform_stamped.transform.translation.x = 0.0;
    //     transform_stamped.transform.translation.y = 0.0;
    //     transform_stamped.transform.translation.z = 0.0;
    // }
    // transform_stamped.transform.rotation.x = m_imu->orientation.x;
    // transform_stamped.transform.rotation.y = m_imu->orientation.y;
    // transform_stamped.transform.rotation.z = m_imu->orientation.z;
    // transform_stamped.transform.rotation.w = m_imu->orientation.w;

    EKFState state = m_sensor_fusion->getState();
    Eigen::Quaterniond body_to_world_orientation
        = Eigen::Quaterniond(state.imu_orientation.w(), state.imu_orientation.x(), state.imu_orientation.y(), state.imu_orientation.z());
    transform_stamped.transform.translation.x = state.imu_position.x();
    transform_stamped.transform.translation.y = state.imu_position.y();
    transform_stamped.transform.translation.z = state.imu_position.z();
    transform_stamped.transform.rotation.x = body_to_world_orientation.x();
    transform_stamped.transform.rotation.y = body_to_world_orientation.y();
    transform_stamped.transform.rotation.z = body_to_world_orientation.z();
    transform_stamped.transform.rotation.w = body_to_world_orientation.w();

    m_tf_broadcaster->sendTransform(transform_stamped);
}


/*******************************************************************************
 * Configuration Functions
 *******************************************************************************/

void StateEstimatorNode::declareParameters()
{
    declare_parameter("simulation", true);
    declare_parameter("urdf_file_path", std::string());
    declare_parameter("robot_definition", std::string());
    declare_parameter("clock_period", 0.05);

    // Thresholds for stance detection
    declare_parameter("left_stance_threshold", 400.0);
    declare_parameter("right_stance_threshold", 400.0);

    // Noise parameters for sensor fusion
    declare_parameter("noise_parameters.process_noise.linear_acceleration", std::vector<double>());
    declare_parameter("noise_parameters.process_noise.angular_velocity", std::vector<double>());
    declare_parameter("noise_parameters.process_noise.foot_position", std::vector<double>());
    declare_parameter("noise_parameters.process_noise.accelerometer_bias", std::vector<double>());
    declare_parameter("noise_parameters.process_noise.gyroscope_bias", std::vector<double>());
    declare_parameter("noise_parameters.process_noise.foot_slippage", std::vector<double>());
    declare_parameter("noise_parameters.observation_noise.foot_position", std::vector<double>());
    declare_parameter("noise_parameters.observation_noise.foot_slippage", std::vector<double>());
    declare_parameter("noise_parameters.observation_noise.joint_position", std::vector<double>());

    RCLCPP_INFO(this->get_logger(), "Parameters have been declared");
}

void StateEstimatorNode::configureSubscriptions()
{
    m_sensors_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_sensors_subscription_options.callback_group = m_sensors_callback_group;

    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states/filtered", rclcpp::SensorDataQoS(),
        std::bind(&StateEstimatorNode::jointStateCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("lower_imu", rclcpp::SensorDataQoS(),
        std::bind(&StateEstimatorNode::imuCallback, this, std::placeholders::_1), m_sensors_subscription_options);

    // Simulation ground truth information about position and velocity in world frame
    if (m_is_simulation) {
        m_imu_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("lower_imu/position", rclcpp::SensorDataQoS(),
            std::bind(&StateEstimatorNode::imuPositionCallback, this, std::placeholders::_1), m_sensors_subscription_options);
        m_imu_velocity_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("lower_imu/velocity", rclcpp::SensorDataQoS(),
            std::bind(&StateEstimatorNode::imuVelocityCallback, this, std::placeholders::_1), m_sensors_subscription_options);
    }

    m_noise_params_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("state_estimation/noise_parameters", 10,
        std::bind(&StateEstimatorNode::noiseParametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriptions have been configured");
}

void StateEstimatorNode::configurePublishers()
{
    m_clock_pub = this->create_publisher<std_msgs::msg::Header>("state_estimation/clock", 10);
    m_state_estimation_pub = this->create_publisher<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10);
    m_feet_height_pub = this->create_publisher<march_shared_msgs::msg::FeetHeightStamped>("state_estimation/feet_height", 10);
    m_torque_left_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("state_estimation/ground_reaction_force/left", 10);
    m_torque_right_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("state_estimation/ground_reaction_force/right", 10);

    // M8's MPC
    m_mpc_foot_positions_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("est_foot_position", 10);
    m_mpc_com_pub = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 10);
    m_mpc_zmp_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 10);
    m_mpc_stance_foot_pub = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 10);
    // Visualization of COM position
    m_mpc_com_pos_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("state_estimation/com_position", 10);

    RCLCPP_INFO(this->get_logger(), "Publishers have been configured");
}

void StateEstimatorNode::configureStateEstimationTimer()
{
    m_dt = get_parameter("clock_period").as_double();
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(m_dt * 1000)), 
        std::bind(&StateEstimatorNode::timerCallback, this), m_sensors_callback_group);
    m_timer->cancel();

    RCLCPP_INFO(this->get_logger(), "State Estimation Timer has been configured. Timestep: %f", m_dt);
}

void StateEstimatorNode::configureTF2()
{
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    RCLCPP_INFO(this->get_logger(), "TF2 broadcaster and listener have been configured");
}

bool StateEstimatorNode::configureJointStateMsg()
{
    urdf::Model model;
    if (!model.initFile(get_parameter("urdf_file_path").as_string())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load URDF file");
        return false;
    }

    // Retrieve joint names from URDF and sort them in alphabetical order.
    std::vector<std::string> joint_names;
    for (const auto& joint : model.joints_) {
        if (joint.second->type != urdf::Joint::REVOLUTE)
            continue;
        joint_names.push_back(joint.first);
    }
    std::sort(joint_names.begin(), joint_names.end());
 
    // Initialize joint state message.
    m_joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    m_joint_state->header.frame_id = "joint_link";
    m_joint_state->header.stamp = this->now();
    m_joint_state->name = joint_names;
    m_joint_state->position = std::vector<double>(joint_names.size(), 0.0);
    m_joint_state->velocity = std::vector<double>(joint_names.size(), 0.0);
    m_joint_state->effort = std::vector<double>(joint_names.size(), 0.0);

    // Initialize last update timeout
    m_joint_state_timeout = 1.0; // s

    RCLCPP_INFO(this->get_logger(), "Joint State message has been configured");
    return true;
}

bool StateEstimatorNode::configureImuMsg()
{
    const double GRAVITIY_ACCELERATION = 9.81;

    // Initialize IMU message.
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
    m_imu->linear_acceleration.z = -GRAVITIY_ACCELERATION;

    // Initialize last update timeout
    m_imu_timeout = 1.0; // s

    RCLCPP_INFO(this->get_logger(), "IMU message has been configured");
    return true;
}

bool StateEstimatorNode::configureStateEstimator()
{
    // Configure robot description
    std::string yaml_filename = get_parameter("robot_definition").as_string();

    if (yaml_filename.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No robot description file has been provided.");
        return false;
    }

    m_robot_description = std::make_shared<RobotDescription>(yaml_filename);

    // Configure state estimator
    std::string urdf_file_path = get_parameter("urdf_file_path").as_string();

    if (urdf_file_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No URDF file path has been provided.");
        return false;
    }

    // Initialize state estimator
    m_node_feet_names = { "L_sole", "R_sole" };
    m_state_estimator = std::make_unique<StateEstimator>(m_robot_description, urdf_file_path);
    m_state_estimator->configureJointNames(m_joint_state->name);
    m_state_estimator->configureStanceThresholds(
        get_parameter("left_stance_threshold").as_double(),
        get_parameter("right_stance_threshold").as_double());

    RCLCPP_INFO(this->get_logger(), "State Estimator has been configured");
    return true;
}

bool StateEstimatorNode::configureSensorFusion()
{
    // Initialize and configure noise parameters in sensor fusion
    m_sensor_fusion = std::make_unique<SensorFusion>(m_dt);
    m_sensor_fusion_valid = true;

    m_sensor_fusion->setProcessNoiseCovarianceMatrix(
        get_parameter("noise_parameters.process_noise.linear_acceleration").as_double_array(),
        get_parameter("noise_parameters.process_noise.angular_velocity").as_double_array(),
        get_parameter("noise_parameters.process_noise.foot_position").as_double_array(),
        get_parameter("noise_parameters.process_noise.accelerometer_bias").as_double_array(),
        get_parameter("noise_parameters.process_noise.gyroscope_bias").as_double_array(),
        get_parameter("noise_parameters.process_noise.foot_slippage").as_double_array());

    m_sensor_fusion->setObservationNoiseCovarianceMatrix(
        get_parameter("noise_parameters.observation_noise.foot_position").as_double_array(),
        get_parameter("noise_parameters.observation_noise.foot_slippage").as_double_array(),
        get_parameter("noise_parameters.observation_noise.joint_position").as_double_array());

    RCLCPP_INFO(this->get_logger(), "Sensor Fusion has been configured");
    return true;
}


/*******************************************************************************
 * Helper functions
 *******************************************************************************/

void StateEstimatorNode::checkJointStateTimeout()
{
    if ((this->now() - m_joint_state_last_update) > rclcpp::Duration::from_seconds(m_joint_state_timeout)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No joint state data received yet");
    }
}

void StateEstimatorNode::checkImuTimeout()
{
    if ((this->now() - m_imu_last_update) > rclcpp::Duration::from_seconds(m_imu_timeout)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No imu data received yet");
    }
}

geometry_msgs::msg::TransformStamped StateEstimatorNode::getCurrentTransform(const std::string& parent_frame, const std::string& child_frame)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = m_tf_buffer->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while getting transform stamped: %s", e.what());
        m_sensor_fusion_valid = false;
    }
    return transform_stamped;
}

geometry_msgs::msg::Pose StateEstimatorNode::getCurrentPose(const std::string& parent_frame, const std::string& child_frame)
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
        m_sensor_fusion_valid = false;
    }
    return pose;
}

std::vector<geometry_msgs::msg::Pose> StateEstimatorNode::getCurrentPoseArray(const std::string& parent_frame, const std::vector<std::string>& child_frames)
{
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const std::string& child_frame : child_frames) {
        poses.push_back(getCurrentPose(parent_frame, child_frame));
    }
    return poses;
}


/*******************************************************************************
 * Main Function
 *******************************************************************************/

int main(int argc, char** argv)
{
    Eigen::initParallel();
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto state_estimator_node = std::make_shared<StateEstimatorNode>();
    executor.add_node(state_estimator_node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}