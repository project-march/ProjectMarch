// standard
#include "c_generated_code/main_pendulum_ode.cpp"
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SolverNode : public rclcpp::Node {
public:
    SolverNode()
        : Node("mpc_solver_node")
        , x_current()
        , m_time_horizon(3.0)
    {
        trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
        robot_state_subscriber = this->create_subscription<march_shared_msgs::msg::RobotState>(
            "robot_state", 10, std::bind(&SolverNode::robot_state_callback, this, _1));
        gait_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "gait", 10, std::bind(&SolverNode::gait_callback, this, _1));
    };
    double x_current[NX];
    double u_current[NU * PENDULUM_ODE_N];
    double m_time_horizon;

private:
    int solve_step(double (&x_cur)[NX], double (&u_cur)[NU * PENDULUM_ODE_N])
    {
        // return solve_mpc(x_cur, u_cur);
        return 0;
    };

    void gait_callback(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        return;
    };

    void robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr msg)
    {
        x_current[0] = msg->joint_pos[0]; // set x position
        x_current[1] = msg->joint_vel[0]; // set theta position
        int status = solve_step(x_current, u_current); // solve the mpc problem
        if (status == 0) {
            publish_control_msg();
        }
    };

    void publish_control_msg()
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
    };
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    rclcpp::Subscription<march_shared_msgs::msg::RobotState>::SharedPtr robot_state_subscriber;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr gait_subscriber;
};

int main(int argc, char** argv)
{
    printf("hello world acados_solver package\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
