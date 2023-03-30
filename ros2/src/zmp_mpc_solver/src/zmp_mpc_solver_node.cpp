// standard
#include "zmp_mpc_solver/zmp_mpc_solver_node.hpp"
//#include "march_shared_msgs/msg/point_stamped_list.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

SolverNode::SolverNode()
    : Node("mpc_solver_node")
    , m_zmp_solver()
{
    //    m_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory",
    //    10);
    m_com_trajectory_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("com_trajectory", 10);
    m_final_feet_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("final_feet_position", 10);
    m_com_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "robot_com_position", 10, std::bind(&SolverNode::com_callback, this, _1));
    m_feet_pos_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "desired_footsteps", 10, std::bind(&SolverNode::feet_callback, this, _1));
    m_zmp_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "robot_zmp_position", 10, std::bind(&SolverNode::zmp_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Booted up ZMP solver node");
}

void SolverNode::com_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    //    m_zmp_solver.set_current_com(
    //        msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[1].position.x, msg->poses[1].position.y);
    RCLCPP_DEBUG(this->get_logger(), "com callback test");
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

// void SolverNode::robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr msg)
//{
//    // int status = solve_step(x_current, u_current); // solve the mpc problem
//    // if (status == 0) {
//    // publish_control_msg();
//    // }
//}

void SolverNode::publish_control_msg()
{
    auto message = geometry_msgs::msg::PoseArray();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "map";
    geometry_msgs::msg::Pose pose_container;


    std::array<double, NX * ZMP_PENDULUM_ODE_N>* trajectory_pointer = m_zmp_solver.get_state_trajectory();

    for (int i = 0; i<(ZMP_PENDULUM_ODE_N); i++) {
        pose_container.position.x = (*trajectory_pointer)[(i*NX + 0)];
        pose_container.position.y = (*trajectory_pointer)[(i*NX + 3)];
        pose_container.position.z = m_zmp_solver.get_com_height();
      message.poses.push_back(pose_container);
    };
    // message.mode = 0;
    // message.control_inputs = 1;
    // message.dt = (m_time_horizon / PENDULUM_ODE_N);
    m_com_trajectory_publisher->publish(message);
    return;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
