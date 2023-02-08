#include "com_estimator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <array>
#include <chrono>
#include <cstdio>
#include <string>

using std::placeholders::_1;
#ifndef STATE_ESTIMATOR
#define STATE_ESTIMATOR

class StateEstimator : public rclcpp::Node {
public:
    StateEstimator();

    sensor_msgs::msg::JointState get_initial_joint_states();

private:
    void sensor_callback(sensor_msgs::msg::Imu::SharedPtr msg);

    void state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    void publish_robot_state();

    void publish_robot_frames();

    rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr m_state_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_sensor_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_state_subscriber;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_joint_tf_broadcaster;
    JointEstimator m_joint_estimator;
    ComEstimator m_com_estimator;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif