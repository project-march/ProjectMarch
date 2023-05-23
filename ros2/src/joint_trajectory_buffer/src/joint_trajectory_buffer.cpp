//
// Created by rixt on 16-5-23.
//

#include "../include/joint_trajectory_buffer/joint_trajectory_buffer.hpp"
#include <chrono>
#include <cstdio>
#include <string>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

TrajectoryBufferNode::TrajectoryBufferNode()
        : Node("joint_trajectory_buffer_node")
{

    m_torque_trajectory_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
            "torque_trajectory", 10, std::bind(&TrajectoryBufferNode::torque_trajectory_callback, this, _1));

    m_position_trajectory_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
            "position_trajectory", 10, std::bind(&TrajectoryBufferNode::position_trajectory_callback, this, _1));

    m_joint_trajectory_publisher
            = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
}

void TrajectoryBufferNode::torque_trajectory_callback(trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "updating the torque...");
    m_torque_buffer.push(msg);
    publish_joint_trajectory();
}

void TrajectoryBufferNode::position_trajectory_callback(trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "updating the position...");
    m_position_buffer.push(msg);
    publish_joint_trajectory();
}

void TrajectoryBufferNode::publish_joint_trajectory()
{
    // check if there is already new information in the buffer and promote the values if so
    if(!m_position_buffer.empty() && !m_torque_buffer.empty()){
        std::get<0>(current) = m_position_buffer.front();
        m_position_buffer.pop();
        std::get<1>(current) = m_torque_buffer.front();
        m_torque_buffer.pop();
    }

    auto pos = std::get<0>(current);
    auto torque = std::get<1>(current);

    // check if both parts of the tuple are filled in
    if(!pos or !torque ){
        // if not, we wait
        RCLCPP_INFO(this->get_logger(), "waiting for input");
        return;
    }
    else{
        RCLCPP_INFO(this->get_logger(), "publishing both... pos %f, torque %f ", pos->positions[0], torque->effort[0]);

        // publish the points
        trajectory_msgs::msg::JointTrajectory joint_trajectory = trajectory_msgs::msg::JointTrajectory();
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = pos->positions;
        point.velocities = pos->velocities;
        point.effort = torque->effort;
        joint_trajectory.points.push_back(point);
        joint_trajectory.header.stamp = this->get_clock()->now();
        m_joint_trajectory_publisher->publish(joint_trajectory);
    }

}