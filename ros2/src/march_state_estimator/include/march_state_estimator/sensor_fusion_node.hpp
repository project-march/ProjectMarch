/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include "march_state_estimator/robot_description.hpp"

class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode(std::shared_ptr<RobotDescription> robot_description);
    ~SensorFusionNode() = default;

private:
    void timerCallback();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishStateEstimation();

    //eventually delete this function 
    void handleGetCurrentJointPositions(std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request>,
        std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response);
        
    void requestNodePositions(const sensor_msgs::msg::JointState::SharedPtr msg);

    double m_dt;
    sensor_msgs::msg::JointState::SharedPtr m_joint_state;
    sensor_msgs::msg::Imu::SharedPtr m_imu;
    std::vector<std::string> m_node_feet_names;
    std::vector<geometry_msgs::msg::Point> m_foot_positions;
    std::shared_ptr<RobotDescription> m_robot_description;
    
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_pub;

    //eventually delete this service
    rclcpp::Service<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr m_get_current_joint_angles_service;

    rclcpp::CallbackGroup::SharedPtr m_joint_state_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_imu_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_timer_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_node_position_callback_group;

    rclcpp::SubscriptionOptions m_joint_state_subscription_options;
    rclcpp::SubscriptionOptions m_imu_subscription_options;
};

#endif  // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_NODE_HPP_