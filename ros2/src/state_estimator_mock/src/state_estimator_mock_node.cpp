//
// Created by Jack Zeng march8 on 30-05-2023
//
#include "state_estimator_mock/state_estimator_mock_node.hpp"
#include "state_estimator_mock/state_estimator_mock.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

StateEstimatorMockNode::StateEstimatorMockNode()
    : Node("state_estimator_mock_node")
    , m_state_estimator_mock()
{
    // subscribers
    m_current_shooting_node_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "current_shooting_node", 10, std::bind(&StateEstimatorMockNode::current_shooting_node_callback, this, _1));

    // publishers
    m_com_pos_publisher = this->create_publisher<march_shared_msgs::msg::CenterOfMass>("robot_com_position", 100);
    m_foot_pos_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("est_foot_position", 100);
    m_zmp_pos_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_zmp_position", 100);
    m_stance_foot_publisher = this->create_publisher<std_msgs::msg::Int32>("current_stance_foot", 100);
    m_right_foot_on_ground_publisher = this->create_publisher<std_msgs::msg::Bool>("right_foot_on_ground", 100);
    m_left_foot_on_ground_publisher = this->create_publisher<std_msgs::msg::Bool>("left_foot_on_ground", 100);
}

void StateEstimatorMockNode::current_shooting_node_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    m_state_estimator_mock.set_current_shooting_node(msg->data);
    RCLCPP_INFO(rclcpp::get_logger(""), "fake shooting node is %i", m_current_shooting_node);
}

void StateEstimatorMockNode::

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