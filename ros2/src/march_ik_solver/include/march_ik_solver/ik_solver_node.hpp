#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "march_ik_solver/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "geometry_msgs/msg/point.hpp"

class IKSolverNode : public rclcpp::Node
{
    public:
        IKSolverNode();

    private:
        void timerCallback();
        void IksFootPositionsCallback(const march_shared_msgs::msg::IksFootPositions::SharedPtr msg);
        void publishJointState(const Eigen::VectorXd joint_config);
        
        IKSolver ik_solver_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<march_shared_msgs::msg::IksFootPositions>::SharedPtr ik_solver_command_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

#endif  // IK_SOLVER__IK_SOLVER_NODE_HPP_