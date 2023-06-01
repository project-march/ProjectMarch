//
// Created by Jack Zeng M8 on 30-05-23.
//

#ifndef STATE_ESTIMATOR_MOCK_NODE_HPP
#define STATE_ESTIMATOR_MOCK_NODE_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_estimator_mock/state_estimator_mock.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstdio>
#include <string>

class StateEstimatorMockNode : public rclcpp::Node {
public:
    StateEstimatorMockNode();
    void publishtrajectories();

private:
    rclcpp::Publisher<march_shared_msgs::msg::CenterOfMass>::SharedPtr m_com_pos_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_zmp_pos_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_foot_pos_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_stance_foot_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_right_foot_on_ground_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_left_foot_on_ground_publisher;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_current_shooting_node_subscriber;

    void current_shooting_node_callback(std_msgs::msg::Int32::SharedPtr msg);
    StateEstimatorMock m_state_estimator_mock;

    rclcpp::TimerBase::SharedPtr m_solving_timer;
};
#endif // BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
