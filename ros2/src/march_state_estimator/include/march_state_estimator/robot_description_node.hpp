#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/msg/state_estimator_visualization.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "march_shared_msgs/srv/get_node_jacobian.hpp"
#include "march_shared_msgs/srv/get_current_stance_leg.hpp"

#include "march_state_estimator/robot_description.hpp"

class RobotDescriptionNode : public rclcpp::Node
{
public:
    RobotDescriptionNode(std::shared_ptr<RobotDescription> robot_description);

private:
    void configureParameters();
    void handleNodePositionRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Response> response);
    void handleNodeJacobianRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Response> response);

    std::shared_ptr<RobotDescription> m_robot_description;
    rclcpp::CallbackGroup::SharedPtr m_node_positions_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_node_jacobian_callback_group;
    rclcpp::Service<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_service_node_position;
    rclcpp::Service<march_shared_msgs::srv::GetNodeJacobian>::SharedPtr m_service_node_jacobian;

};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_NODE_HPP_