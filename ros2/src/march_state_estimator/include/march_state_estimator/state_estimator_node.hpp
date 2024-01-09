#ifndef MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"

class StateEstimatorNode : public rclcpp::Node
{
public:
    StateEstimatorNode();
    ~StateEstimatorNode() = default;

private:
    void timerCallback();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void nodePositionCallback(
        const rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedFuture future);
    void publishStateEstimation();
    void requestNodePositions(const sensor_msgs::msg::JointState::SharedPtr msg);

    double m_dt;
    sensor_msgs::msg::JointState::SharedPtr m_joint_state;
    sensor_msgs::msg::Imu::SharedPtr m_imu;
    std::vector<std::string> m_node_feet_names;
    std::vector<geometry_msgs::msg::Point> m_foot_positions;
    
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_pub;

    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_get_node_position_client;
    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedFuture m_get_node_position_future;
    march_shared_msgs::srv::GetNodePosition::Request::SharedPtr m_get_node_position_request;

    rclcpp::CallbackGroup::SharedPtr m_joint_state_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_imu_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_timer_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_node_position_callback_group;

    rclcpp::SubscriptionOptions m_joint_state_subscription_options;
    rclcpp::SubscriptionOptions m_imu_subscription_options;
};

#endif  // MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_