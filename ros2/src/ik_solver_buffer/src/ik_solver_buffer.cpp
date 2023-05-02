#include "ik_solver_buffer/ik_solver_buffer.hpp"
#include <chrono>
#include <cstdio>
#include <string>
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

BufferNode::BufferNode()
    : Node("trajectory_buffer")
{
    m_com_trajectory_publisher
        = this->create_publisher<march_shared_msgs::msg::IkSolverCommand>("ik_solver_com_input", 10);
    m_swing_trajectory_publisher
        = this->create_publisher<march_shared_msgs::msg::IkSolverCommand>("ik_solver_swing_input", 10);

    m_com_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/com_trajectory", 10, std::bind(&BufferNode::com_subscriber_callback, this, _1));
    m_swing_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/bezier_trajectory", 10, std::bind(&BufferNode::swing_subscriber_callback, this, _1));
    // Initializing the timestep in ms
    declare_parameter("timestep", 1000);
    m_timestep = this->get_parameter("timestep").as_int();
}

void BufferNode::com_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    set_com_trajectory(msg);
    publish_com_trajectory();
}

void BufferNode::swing_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    set_swing_trajectory(msg);
    publish_swing_trajectory();
}

void BufferNode::set_com_trajectory(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_com_trajectory = setter;
}

void BufferNode::set_velocity(
    std::vector<geometry_msgs::msg::Point>& position_vector, std::vector<geometry_msgs::msg::Point>& output_vector)
{

    if (position_vector.size() > 1) {
        geometry_msgs::msg::Point point_container;
        geometry_msgs::msg::Point point_prev = position_vector[0];

        for (auto it = std::begin(position_vector) + 1; it != std::end(position_vector); it++) {
            point_container.x = (it->x - point_prev.x) / (m_timestep * 1e-3);
            point_container.y = (it->y - point_prev.y) / (m_timestep * 1e-3);
            point_container.z = (it->z - point_prev.z) / (m_timestep * 1e-3);
            output_vector.push_back(point_container);
        }

        output_vector.push_back(point_container);
    } else {
        RCLCPP_WARN(this->get_logger(), "trajectory not long enough, cannot determine velocity");
    }
}

void BufferNode::set_swing_trajectory(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_swing_trajectory = setter;
}

bool BufferNode::check_if_ready()
{
    return ((m_latest_com_trajectory) && (m_latest_swing_trajectory));
}

void BufferNode::publish_com_trajectory()
{
    march_shared_msgs::msg::IkSolverCommand ik_command_to_send;
    for (auto i : m_latest_com_trajectory->poses) {
        ik_command_to_send.trajectory.push_back(i.position);
    }

    set_velocity(ik_command_to_send.trajectory, ik_command_to_send.velocity);

    m_com_trajectory_publisher->publish(ik_command_to_send);

    // Reset all the pointers
    m_latest_com_trajectory.reset();
}

void BufferNode::publish_swing_trajectory()
{
    march_shared_msgs::msg::IkSolverCommand ik_command_to_send;
    march_shared_msgs::msg::IkSolverCommand ik_mock_com;
    for (auto i : m_latest_swing_trajectory->poses) {
        ik_command_to_send.trajectory.push_back(i.position);
        geometry_msgs::msg::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        ik_mock_com.trajectory.push_back(p);
    }

    set_velocity(ik_command_to_send.trajectory, ik_command_to_send.velocity);
    set_velocity(ik_mock_com.trajectory, ik_mock_com.velocity);

    m_swing_trajectory_publisher->publish(ik_command_to_send);
    m_com_trajectory_publisher->publish(ik_mock_com);

    // Reset all the pointers
    m_latest_swing_trajectory.reset();
}
