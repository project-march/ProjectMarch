#include "march_ik_solver/ik_solver_point_evaluator_node.hpp"

#include <memory>
#include <string>
#include <vector>

IKSolverPointEvaluatorNode::IKSolverPointEvaluatorNode() : Node("ik_solver_point_evaluator_node")
{
    declare_parameter("desired_left_foot_position.x", 0.0);
    declare_parameter("desired_left_foot_position.y", 0.0);
    declare_parameter("desired_left_foot_position.z", 0.0);
    declare_parameter("desired_right_foot_position.x", 0.0);
    declare_parameter("desired_right_foot_position.y", 0.0);
    declare_parameter("desired_right_foot_position.z", 0.0);

    m_desired_left_foot_position.x = get_parameter("desired_left_foot_position.x").as_double();
    m_desired_left_foot_position.y = get_parameter("desired_left_foot_position.y").as_double();
    m_desired_left_foot_position.z = get_parameter("desired_left_foot_position.z").as_double();
    m_desired_right_foot_position.x = get_parameter("desired_right_foot_position.x").as_double();
    m_desired_right_foot_position.y = get_parameter("desired_right_foot_position.y").as_double();
    m_desired_right_foot_position.z = get_parameter("desired_right_foot_position.z").as_double();

    m_state_estimation_sub = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 1, std::bind(&IKSolverPointEvaluatorNode::stateEstimationCallback, this, std::placeholders::_1));
    m_iks_foot_positions_pub = this->create_publisher<march_shared_msgs::msg::IksFootPositions>(
        "ik_solver/buffer/input", 10);
}

void IKSolverPointEvaluatorNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    (void) msg;
    publishIKSFootPositions();
}

void IKSolverPointEvaluatorNode::publishIKSFootPositions()
{
    march_shared_msgs::msg::IksFootPositions iks_foot_positions_msg;
    iks_foot_positions_msg.header.stamp = this->now();
    iks_foot_positions_msg.header.frame_id = "base_link";
    iks_foot_positions_msg.left_foot_position = m_desired_left_foot_position;
    iks_foot_positions_msg.right_foot_position = m_desired_right_foot_position;

    RCLCPP_DEBUG(this->get_logger(), "Publishing desired foot positions");
    RCLCPP_DEBUG(this->get_logger(), "Left foot position: x: %f, y: %f, z: %f", m_desired_left_foot_position.x,
                m_desired_left_foot_position.y, m_desired_left_foot_position.z);
    RCLCPP_DEBUG(this->get_logger(), "Right foot position: x: %f, y: %f, z: %f", m_desired_right_foot_position.x,
                m_desired_right_foot_position.y, m_desired_right_foot_position.z);

    m_iks_foot_positions_pub->publish(iks_foot_positions_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSolverPointEvaluatorNode>());
    rclcpp::shutdown();
    return 0;
}