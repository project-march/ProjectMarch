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

    m_com_subscriber = this->create_subscription<march_shared_msgs::msg::CenterOfMass>(
        "/robot_com_position", 10, std::bind(&SolverNode::com_callback, this, _1));
    m_feet_pos_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/desired_footsteps", 10, std::bind(&SolverNode::feet_callback, this, _1));
    m_zmp_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/robot_zmp_position", 10, std::bind(&SolverNode::zmp_callback, this, _1));
    m_stance_foot_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "/current_stance_foot", 10, std::bind(&SolverNode::stance_foot_callback, this, _1));

    m_solving_timer = this->create_wall_timer(8ms, std::bind(&SolverNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Booted up ZMP solver node");
}

void SolverNode::com_callback(march_shared_msgs::msg::CenterOfMass::SharedPtr msg)
{
    m_zmp_solver.set_current_com(msg->position.x, msg->position.y, msg->velocity.x, msg->velocity.y);
    m_zmp_solver.set_com_height(msg->position.z);
}

void SolverNode::zmp_callback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_zmp_solver.set_current_zmp(msg->point.x, msg->point.y);
}

void SolverNode::feet_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    m_zmp_solver.set_current_foot(msg->poses[1].position.x, msg->poses[1].position.y);
    m_zmp_solver.set_previous_foot(msg->poses[0].position.x, msg->poses[0].position.y);

    // ADD CANDIDATE FOOTSTEPS HERE
    m_zmp_solver.set_candidate_footsteps(msg);
    m_zmp_solver.set_reference_stepsize(m_zmp_solver.get_candidate_footsteps());
}

void SolverNode::stance_foot_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_zmp_solver.set_current_stance_foot(msg->data);
}

// void SolverNode::robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr msg)
//{
//    // int status = solve_step(x_current, u_current); // solve the mpc problem
//    // if (status == 0) {
//    // publish_control_msg();
//    // }
//}

void SolverNode::timer_callback()
{
    m_zmp_solver.set_current_state();
    int solver_status = m_zmp_solver.solve_step();
    if (solver_status != 0)
        {
        RCLCPP_WARN(this->get_logger(), "Could not find a solution. exited with status %i", solver_status);
        }
    auto com_msg = geometry_msgs::msg::PoseArray();
    com_msg.header.stamp = this->get_clock()->now();
    com_msg.header.frame_id = "map";

    auto foot_msg = geometry_msgs::msg::PoseArray();
    foot_msg.header.stamp = this->get_clock()->now();
    foot_msg.header.frame_id = "map";

    geometry_msgs::msg::Pose pose_container;

    std::array<double, NX* ZMP_PENDULUM_ODE_N>* trajectory_pointer = m_zmp_solver.get_state_trajectory();

    for (int i = 0; i < (ZMP_PENDULUM_ODE_N); i++) {
        pose_container.position.x = (*trajectory_pointer)[(i * NX + 0)];
        pose_container.position.y = (*trajectory_pointer)[(i * NX + 3)];
        pose_container.position.z = m_zmp_solver.get_com_height();
        com_msg.poses.push_back(pose_container);

        // The feet
        pose_container.position.x = (*trajectory_pointer)[(i * NX + 6)];
        pose_container.position.y = (*trajectory_pointer)[(i * NX + 8)];
        foot_msg.poses.push_back(pose_container);
    };
    m_com_trajectory_publisher->publish(com_msg);
    m_final_feet_publisher->publish(foot_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
