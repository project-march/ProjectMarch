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
        : Node("joint_trajectory_buffer")
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
    m_torque_buffer = msg;
    publish_joint_trajectory();
}

void TrajectoryBufferNode::position_trajectory_callback(trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    m_position_buffer = msg;
    publish_joint_trajectory();
}

void TrajectoryBufferNode::publish_joint_trajectory()
{

    // check if there is already new information in the buffer and promote the values if so
    if(m_position_buffer && m_torque_buffer){
        std::get<0>(current) = m_position_buffer;
        std::get<1>(current) = m_torque_buffer;
        m_position_buffer = NULL;
        m_torque_buffer = NULL;
    }

    auto pos = std::get<0>(current);
    auto torque = std::get<1>(current);

    // check if both parts of the tuple are filled in
    if(!pos or !torque ){
        // if not, we wait
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for input...");
    }

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