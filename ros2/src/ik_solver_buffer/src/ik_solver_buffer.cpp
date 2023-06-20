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

/**
 * This callback receives the con trajectory fromm the zmp-mpc.
 * @param msg
 */
void BufferNode::com_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    set_com_trajectory(msg);
    publish_com_trajectory();
}

/**
 * This callback processes the swing leg trajectory from the swi ng leg generator.
 * @param msg
 */
void BufferNode::swing_subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    set_swing_trajectory(msg);
    publish_swing_trajectory();
}

/**
 * This function sets the com trajectory in the ik_buffer to the latest received trajectory.
 * @param setter Com trajectory to set.
 */
void BufferNode::set_com_trajectory(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_com_trajectory = setter;
}

/**
 * Set the velocities of the received trajectories. The IK needs position and velocity trajectories to solve correctly.
 * @param position_vector the vector of the position trajectory
 * @param output_vector the vector with the calculated velocities.
 */
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
            point_prev.x = it->x;
            point_prev.y = it->y;
            point_prev.z = it->z;
        }

        output_vector.push_back(point_container);
    } else {
        RCLCPP_WARN(this->get_logger(), "trajectory not long enough, cannot determine velocity");
    }
}

/**
 * Set the swing leg trajectory to the latest received trajectory.
 * @param setter
 */
void BufferNode::set_swing_trajectory(geometry_msgs::msg::PoseArray::SharedPtr setter)
{
    m_latest_swing_trajectory = setter;
}

/**
 * Old function to make the buffer synchronous,
 *
 * NOTE: This function is not used now, since we send the com trajectory more often than the swing trajectory.
 * @return
 */
bool BufferNode::check_if_ready()
{
    return ((m_latest_com_trajectory) && (m_latest_swing_trajectory));
}

/**
 * Publish the com trajectory.
 * Before this happens, the trajectory should also get a velocity trajectory.
 * When that is done the trajectory is send to the IK solver
 */
void BufferNode::publish_com_trajectory()
{
    march_shared_msgs::msg::IkSolverCommand ik_command_to_send;
    for (auto i : m_latest_com_trajectory->poses) {
        ik_command_to_send.trajectory.push_back(i.position);
    }

    set_velocity(ik_command_to_send.trajectory, ik_command_to_send.velocity);

    // Add a final 0 point
    geometry_msgs::msg::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    ik_command_to_send.velocity.push_back(p);

    m_com_trajectory_publisher->publish(ik_command_to_send);

    // Reset all the pointers
    m_latest_com_trajectory.reset();
}

/**
 * Publish the swing trajectory.
 * Before this happens, the trajectory should also get a velocity trajectory.
 * When that is done the trajectory is send to the IK solver
 */
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

    // Add a final 0 point
    geometry_msgs::msg::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    ik_command_to_send.velocity.push_back(p);

    m_swing_trajectory_publisher->publish(ik_command_to_send);

    // Reset all the pointers
    m_latest_swing_trajectory.reset();
}
