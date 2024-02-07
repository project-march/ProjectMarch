#ifndef MARCH_IK_SOLVER__IK_SOLVER_POINT_EVALUATOR_NODE_HPP
#define MARCH_IK_SOLVER__IK_SOLVER_POINT_EVALUATOR_NODE_HPP

#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "rclcpp/rclcpp.hpp"

class IKSolverPointEvaluatorNode : public rclcpp::Node {
public:
    IKSolverPointEvaluatorNode();
    ~IKSolverPointEvaluatorNode() = default;

private:
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void publishIKSFootPositions();

    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;
    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_iks_foot_positions_pub;

    geometry_msgs::msg::Point m_desired_left_foot_position;
    geometry_msgs::msg::Point m_desired_right_foot_position;
};

#endif // MARCH_IK_SOLVER__IK_SOLVER_POINT_EVALUATOR_NODE_HPP