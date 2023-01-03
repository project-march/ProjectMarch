// NOLINTBEGIN
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>
#include <cstdio>
#include <string>

// acados
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

using std::placeholders::_1;

class SolverNode : public rclcpp::Node {
    // TODO The solver node now only passes the given trajectory to without modification.
    // This needs to be changed when the MPC is implemented more.
public:
    SolverNode()
        : Node("solver_node")
    {
            robot_state_subscriber = this->create_subscription<march_shared_msgs::msg::RobotState>(
                 "robot_state", 10, std::bind(&SolverNode::robot_state_callback, this, _1));
             gait_subscriber = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                 "gait", 10, std::bind(&SolverNode::gait_callback, this, _1));
             trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory",
             10);
    };

private:
    void robot_state_callback(march_shared_msgs::msg::RobotState::SharedPtr msg)
    {
        return;
    };

    void gait_callback(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        trajectory_publisher->publish(*msg);
    };

    rclcpp::Subscription<march_shared_msgs::msg::RobotState>::SharedPtr robot_state_subscriber;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr gait_subscriber;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolverNode>());
    rclcpp::shutdown();
    return 0;
}
// NOLINTEND
