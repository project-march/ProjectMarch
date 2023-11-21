//
// Created by Jack Zeng march8 on 30-05-2023
//
#include "state_estimator_mock/state_estimator_mock_node.hpp"
#include "state_estimator_mock/state_estimator_mock.hpp"
#include <cmath>
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

StateEstimatorMockNode::StateEstimatorMockNode()
    : Node("state_estimator_mock_node")
    , m_state_estimator_mock()
{
    // subscribers
    m_current_shooting_node_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "/current_shooting_node", 10, std::bind(&StateEstimatorMockNode::current_shooting_node_callback, this, _1));
    m_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&StateEstimatorMockNode::state_callback, this, _1));

    // publishers
    m_com_pos_publisher = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 100);
    m_stance_foot_publisher = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 100);
    m_right_foot_on_ground_publisher = this->create_publisher<std_msgs::msg::Bool>("right_foot_on_ground", 100);
    m_left_foot_on_ground_publisher = this->create_publisher<std_msgs::msg::Bool>("left_foot_on_ground", 100);
    m_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("measured_joint_states", 100);

    m_solving_timer = this->create_wall_timer(50ms, std::bind(&StateEstimatorMockNode::publishtrajectories, this));
}

void StateEstimatorMockNode::state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{
    // for(auto& i : msg->position){
    //     if (i == 0.0){
    //         return;
    //     }
    // }
    // this->m_joint_estimator.set_joint_states(msg);
    // m_joint_estimator.set_individual_joint_state("right_knee", 0.5);
    m_joint_state_publisher->publish(*msg);
}

void StateEstimatorMockNode::current_shooting_node_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_state_estimator_mock.set_current_shooting_node(msg->data);
    // RCLCPP_INFO(rclcpp::get_logger(""), "fake shooting node is %i",
    // m_state_estimator_mock.get_current_shooting_node()); publishtrajectories();
}

void StateEstimatorMockNode::publishtrajectories()
{

    m_com_pos_publisher->publish(m_state_estimator_mock.get_current_com());
    m_stance_foot_publisher->publish(m_state_estimator_mock.get_current_stance_foot());
    m_right_foot_on_ground_publisher->publish(m_state_estimator_mock.get_right_foot_ground());
    m_left_foot_on_ground_publisher->publish(m_state_estimator_mock.get_left_foot_ground());
}

/**
 * Main function to run the node.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorMockNode>());
    rclcpp::shutdown();
    return 0;
}