/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

#include "march_state_estimator/robot_description.hpp"
#include "march_state_estimator/sensor_fusion.hpp"
#include "march_state_estimator/torque_converter.hpp"

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode(std::shared_ptr<RobotDescription> robot_description);
    ~SensorFusionNode();

private:
    void timerCallback();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void imuPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void publishStateEstimation();
    void publishFeetHeight();
    void publishMPCEstimation();

    double m_dt;
    sensor_msgs::msg::JointState::SharedPtr m_joint_state;
    sensor_msgs::msg::Imu::SharedPtr m_imu;
    geometry_msgs::msg::PointStamped::SharedPtr m_imu_position;
    std::vector<std::string> m_node_feet_names;
    std::vector<geometry_msgs::msg::Point> m_foot_positions;

    std::unique_ptr<SensorFusion> m_sensor_fusion;
    std::shared_ptr<RobotDescription> m_robot_description; // TODO: TO be obtained from SensorFusion.
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_imu_position_sub;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_pub;
    rclcpp::Publisher<march_shared_msgs::msg::FeetHeightStamped>::SharedPtr m_feet_height_pub;

    // M8's MPC
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_mpc_foot_positions_pub;
    rclcpp::Publisher<march_shared_msgs::msg::CenterOfMass>::SharedPtr m_mpc_com_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_mpc_zmp_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_mpc_stance_foot_pub;

    rclcpp::CallbackGroup::SharedPtr m_sensors_callback_group;
    rclcpp::SubscriptionOptions m_sensors_subscription_options;
};

#endif // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_NODE_HPP_