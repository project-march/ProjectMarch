#include "ik_solver_buffer/ik_solver_buffer.hpp"
#include <cstdio>

#include <chrono>
#include <cstdio>
#include <string>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

BufferNode::BufferNode()
    : Node("trajectory_buffer")
{
    m_buffer_publisher = this->create_publisher<march_shared_msgs::msg::IkSolverCommand>("ik_solver_input", 10);

    m_com_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/com_trajectory", 10, std::bind(&BufferNode::com_subscriber_callback, this, _1));
    m_swing_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/bezier_trajectory", 10, std::bind(&BufferNode::swing_subscriber_callback, this, _1));
}

void BufferNode::com_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    set_com_trajectory(msg);
    if (check_if_ready()) {
        publish_ik_trajectory();
    }
}

void BufferNode::swing_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    set_swing_trajectory(msg);
    if (check_if_ready()) {
        publish_ik_trajectory();
    }
}

void BufferNode::set_com_trajectory(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_com_trajectory = setter;
}

void BufferNode::set_swing_trajectory(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_swing_trajectory = setter;
}

bool BufferNode::check_if_ready()
{
    return ((m_latest_com_trajectory) && (m_latest_swing_trajectory));
}

void BufferNode::publish_ik_trajectory()
{
    march_shared_msgs::msg::IkSolverCommand ik_command_to_send;
    for (auto i : m_latest_com_trajectory->poses) {
        ik_command_to_send.com_trajectory.push_back(i.position);
    }

    for (auto i : m_latest_swing_trajectory->poses) {
        ik_command_to_send.swing_trajectory.push_back(i.position);
    }

    m_buffer_publisher->publish(ik_command_to_send);

    // Reset all the pointers
    m_latest_com_trajectory.reset();
    m_latest_swing_trajectory.reset();
}
