#ifndef MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"


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
    void requestNodePositions();
    void handleGetCurrentJointPositions(std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request>,
        std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response);

    double m_dt;
    sensor_msgs::msg::JointState::SharedPtr m_joint_state;
    sensor_msgs::msg::Imu::SharedPtr m_imu;
    std::vector<geometry_msgs::msg::Point> m_foot_positions;
    
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_pub;

    std::vector<std::string> m_node_feet_names;

    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_get_node_position_client;
    rclcpp::Client<march_shared_msgs::srv::GetNodePosition>::SharedFuture m_get_node_position_future;
    march_shared_msgs::srv::GetNodePosition::Request::SharedPtr m_get_node_position_request;
    rclcpp::Service<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr m_get_current_joint_angles_service;

};

#endif  // MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_