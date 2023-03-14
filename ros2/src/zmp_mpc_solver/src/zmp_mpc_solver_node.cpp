// standard
#include "zmp_mpc_solver/zmp_mpc_solver_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

SolverNode::SolverNode()
    : Node("mpc_solver_node")
    , m_zmp_solver()
{
    m_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);
    m_com_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "robot_com_position", 10, std::bind(&SolverNode::com_callback, this, _1));
    m_feet_pos_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "robot_feet_positions", 10, std::bind(&SolverNode::feet_callback, this, _1));
    m_zmp_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "robot_zmp_position", 10, std::bind(&SolverNode::zmp_callback, this, _1));
}

void SolverNode::com_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    m_zmp_solver.set_current_com(
        msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[1].position.x, msg->poses[1].position.y);
}

void SolverNode::zmp_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "zmp callback test");
}

void SolverNode::feet_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    m_zmp_solver.set_current_foot(msg->poses[1].position.x, msg->poses[1].position.y);
    m_zmp_solver.set_previous_foot(msg->poses[0].position.x, msg->poses[0].position.y);
}

void SolverNode::robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr msg)
{
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

int main(int argc, char** argv)
{
    printf("hello world acados_solver package\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
