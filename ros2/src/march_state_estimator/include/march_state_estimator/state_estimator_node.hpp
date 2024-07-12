/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/sensor_fusion_noise_params.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "march_state_estimator/state_estimator.hpp"
#include "march_state_estimator/sensor_fusion.hpp"
#include "march_state_estimator/torque_converter.hpp"

#define LEFT_FOOT_ID    0
#define RIGHT_FOOT_ID   1

class StateEstimatorNode : public rclcpp_lifecycle::LifecycleNode {
public:
    StateEstimatorNode();
    ~StateEstimatorNode();

private:
    // Lifecycle node callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

    // Subscription callbacks
    void timerCallback();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void imuPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void imuVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void noiseParametersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Publisher functions
    void publishClock();
    void publishStateEstimation();
    void publishFeetHeight();
    void publishMPCEstimation();
    void publishGroundReactionForce();
    void broadcastTransformToTf2();

    // Configuration functions
    void declareParameters();
    void configureSubscriptions();
    void configurePublishers();
    void configureStateEstimationTimer();
    void configureTF2();
    bool configureJointStateMsg();
    bool configureImuMsg();
    bool configureStateEstimator();
    bool configureSensorFusion();

    // Helper functions
    inline bool isInActiveState() { return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE; }
   
    inline void updateJointStateTimeout() { m_joint_state_last_update = now(); }
    inline void updateImuTimeout() { m_imu_last_update = now(); }
    void updateKalmanFilter(const double& dt);
    void checkJointStateTimeout();
    void checkImuTimeout();
   
    geometry_msgs::msg::TransformStamped getCurrentTransform(const std::string& parent_frame, const std::string& child_frame);
    geometry_msgs::msg::Pose getCurrentPose(const std::string& parent_frame, const std::string& child_frame);
    std::vector<geometry_msgs::msg::Pose> getCurrentPoseArray(const std::string& parent_frame, const std::vector<std::string>& child_frames);

    // Member variables
    double m_dt;
    bool m_is_simulation;
    bool m_sensor_fusion_valid;

    sensor_msgs::msg::JointState::SharedPtr m_joint_state;
    sensor_msgs::msg::Imu::SharedPtr m_imu;
    geometry_msgs::msg::PointStamped::SharedPtr m_imu_position;
    geometry_msgs::msg::Vector3Stamped::SharedPtr m_imu_velocity;
    std::vector<geometry_msgs::msg::Point> m_foot_positions;

    std::unique_ptr<StateEstimator> m_state_estimator;
    std::unique_ptr<SensorFusion> m_sensor_fusion;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;

    std::vector<Eigen::Vector3d> m_force_feet;

    double m_joint_state_timeout;
    double m_imu_timeout;
    rclcpp::Time m_joint_state_last_update;
    rclcpp::Time m_imu_last_update;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_imu_position_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr m_imu_velocity_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_noise_params_sub;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Header>::SharedPtr m_clock_pub;
    rclcpp_lifecycle::LifecyclePublisher<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_pub;
    rclcpp_lifecycle::LifecyclePublisher<march_shared_msgs::msg::FeetHeightStamped>::SharedPtr m_feet_height_pub;

    // M8's MPC
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr m_mpc_foot_positions_pub;
    rclcpp_lifecycle::LifecyclePublisher<march_shared_msgs::msg::CenterOfMass>::SharedPtr m_mpc_com_pub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr m_mpc_zmp_pub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr m_mpc_stance_foot_pub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr m_mpc_com_pos_pub;

    // Monitoring topics
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr m_torque_left_pub;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr m_torque_right_pub;

    rclcpp::CallbackGroup::SharedPtr m_sensors_callback_group;
    rclcpp::SubscriptionOptions m_sensors_subscription_options;
};

#endif // MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_