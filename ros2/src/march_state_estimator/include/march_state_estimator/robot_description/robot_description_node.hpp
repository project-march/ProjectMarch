/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION__ROBOT_DESCRIPTION_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION__ROBOT_DESCRIPTION_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/msg/state_estimator_visualization.hpp"
#include "march_shared_msgs/srv/get_current_stance_leg.hpp"
#include "march_shared_msgs/srv/get_node_jacobian.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "march_state_estimator/robot_description/robot_description.hpp"

class RobotDescriptionNode : public rclcpp::Node {
public:
    RobotDescriptionNode();
    ~RobotDescriptionNode();

private:
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void publishVisualization(const std::unordered_map<std::string, double>& joint_positions);
    void handleNodePositionRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Response> response);
    void handleNodeJacobianRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Response> response);

    std::shared_ptr<RobotDescription> m_robot_description;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_subscription_state_estimation;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimatorVisualization>::SharedPtr
        m_publisher_state_estimator_visualization;
    rclcpp::CallbackGroup::SharedPtr m_service_callback_group;
    rclcpp::Service<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_service_node_position;
    rclcpp::Service<march_shared_msgs::srv::GetNodeJacobian>::SharedPtr m_service_node_jacobian;
};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION__ROBOT_DESCRIPTION_NODE_HPP_