#include "ik_solver_buffer/ik_solver_buffer_node.hpp"
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
        "/robot_com_position", 10, std::bind(&BufferNode::com_subscriber_callback, this, _1));
    m_swing_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/bezier_trajectory", 10, std::bind(&BufferNode::swing_subscriber_callback, this, _1));
    m_foot_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/est_foot_position", 10, std::bind(&BufferNode::foot_subscriber_callback, this, _1));
}

void BufferNode::com_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    m_latest_com_trajectory = msg;
    publish_ik_trajectory();
}

void BufferNode::swing_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    m_latest_swing_trajectory = msg;
    publish_ik_trajectory();
}

void BufferNode::foot_subscriber_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_latest_placed_foot = msg;
    publish_ik_trajectory();
}

bool BufferNode::check_if_ready()
{
    return ((m_latest_com_trajectory) && (m_latest_swing_trajectory) && (m_latest_placed_foot));
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

    ik_command_to_send.foot_position = m_latest_placed_foot->point;
    m_buffer_publisher->publish(ik_command_to_send);

    // Reset all the pointers
    m_latest_com_trajectory.reset();
    m_latest_swing_trajectory.reset();
    m_latest_placed_foot.reset();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BufferNode>());
    rclcpp::shutdown();
    return 0;
}
