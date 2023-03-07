// standard
#include "zmp_mpc_solver/zmp_mpc_solver.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

SolverNode::SolverNode()
        : Node("mpc_solver_node")
        , x_current()
        , m_time_horizon(3.0)
    {
        trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
        robot_state_subscriber = this->create_subscription<march_shared_msgs::msg::RobotState>(
            "robot_state", 10, std::bind(&SolverNode::robot_state_callback, this, _1));
        gait_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "gait", 10, std::bind(&SolverNode::gait_callback, this, _1));
    }

int SolverNode::solve_step(&double x_cur[], &double &u_cur[])
    {
        return solve_mpc(x_cur, u_cur);
    }

void SolverNode::gait_callback(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        return;
    }

void SolverNode::robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr msg)
{
    x_current[0] = msg->joint_pos[0]; // set x position
    x_current[1] = msg->joint_vel[0]; // set theta position
    // int status = solve_step(x_current, u_current); // solve the mpc problem
    // if (status == 0) {
        // publish_control_msg();
    // }
}

void SolverNode::publish_control_msg()
{
    // auto message = trajectory_msgs::msg::JointTrajectory();
    // message.header.stamp = this->get_clock()->now();
    // for (const double &control_input : u_current) {
    //   std::cout << control_input << std::endl;
    //   message.reference_control.push_back(control_input);
    // };
    // message.mode = 0;
    // message.control_inputs = 1;
    // message.dt = (m_time_horizon / PENDULUM_ODE_N);
    // m_publisher->publish(message);
    return;
}