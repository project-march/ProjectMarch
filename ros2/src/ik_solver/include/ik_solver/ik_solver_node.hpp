#ifndef IK_SOLVER__IK_SOLVER_NODE_HPP_
#define IK_SOLVER__IK_SOLVER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ik_solver/ik_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class IKSolverNode : public rclcpp::Node
{
    public:
        IKSolverNode();

    private:
        void timer_callback();
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        
        IKSolver ik_solver_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std::vector<std::string> joint_configs_;
        std::string model_name_;
};

#endif  // IK_SOLVER__IK_SOLVER_NODE_HPP_