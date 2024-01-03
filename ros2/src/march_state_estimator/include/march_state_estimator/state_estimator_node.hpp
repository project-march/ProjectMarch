#ifndef MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"

class StateEstimatorNode : public rclcpp::Node
{
public:
    StateEstimatorNode();
    ~StateEstimatorNode() = default;

private:
    void timerCallback();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishStateEstimation();

    int64_t m_dt;
    sensor_msgs::msg::JointState::SharedPtr m_joint_state;
    sensor_msgs::msg::Imu::SharedPtr m_imu;
    
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_pub;
};

#endif  // MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_NODE_HPP_