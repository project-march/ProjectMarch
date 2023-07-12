//
// Created by Tessl Huibregtsen M8 on 7-7-23.
//

#ifndef WEIGHT_SHIFT_BUFFER_NODE_HPP
#define WEIGHT_SHIFT_BUFFER_NODE_HPP
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "march_shared_msgs/msg/current_gait.hpp"
#include "march_shared_msgs/msg/gait_instruction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "weight_shift_buffer/weight_shift_buffer.hpp"
#include <chrono>
#include <cstdio>
#include <string>

class WeightShiftBufferNode : public rclcpp::Node {
public:
    WeightShiftBufferNode();

private:
    WeightShiftBuffer m_weight_shift_buffer;
    
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr m_gait_loader_server;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr m_joint_controller_client;
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID&, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal>);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>);
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>);

    void request_feedback(control_msgs::action::FollowJointTrajectory::Goal);

    void goal_response_callback(
        std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr>);
    void feedback_callback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback>);
    void result_callback(
        const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult&);

    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> m_server_goal_handle;
    std::shared_ptr<control_msgs::action::FollowJointTrajectory::Feedback> m_feedback_pointer;
    std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> m_result_pointer;

    //
    rclcpp::Subscription<march_shared_msgs::msg::GaitInstruction>::SharedPtr m_gait_type_subscriber;
    int m_gait_type;

    void gait_type_callback(march_shared_msgs::msg::GaitInstruction::SharedPtr);
};
#endif // BUILD_WEIGHT_SHIFT_BUFFER_NODE_HPP
