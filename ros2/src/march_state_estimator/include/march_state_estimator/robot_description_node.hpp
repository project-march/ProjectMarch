#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "march_shared_msgs/msg/state_estimator_visualization.hpp"
#include "march_shared_msgs/srv/get_task_report.hpp"
#include "march_shared_msgs/srv/get_node_position.hpp"
#include "march_shared_msgs/srv/get_node_jacobian.hpp"
#include "march_shared_msgs/srv/get_current_stance_leg.hpp"

#include "march_state_estimator/robot_description.hpp"

class RobotDescriptionNode : public rclcpp::Node
{
public:
    RobotDescriptionNode();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishNodePositions();
    void handleTaskReportRequest(const std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Response> response);
    void handleNodePositionRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Response> response);
    void handleNodeJacobianRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Response> response);

    rclcpp::CallbackGroup::SharedPtr m_node_positions_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_node_jacobian_callback_group;

    std::shared_ptr<RobotDescription> m_robot_description;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_subscription;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimatorVisualization>::SharedPtr m_node_positions_publisher;
    // rclcpp::TimerBase::SharedPtr timer_;

    // IMPLEMENTATION: persistent ROS2 services
    // IMPLEMENTATION: function to set up ROS2 services once and persistently connect
    // SOURCE: http://wiki.ros.org/roscpp/Overview/Services#Persistent_Connections
    // IMPLEMENTATION: function of ronnection logic in case of connection failures

    sensor_msgs::msg::JointState::SharedPtr m_joint_state_msg;

    rclcpp::Service<march_shared_msgs::srv::GetTaskReport>::SharedPtr m_service_task_report;
    rclcpp::Service<march_shared_msgs::srv::GetNodePosition>::SharedPtr m_service_node_position;
    rclcpp::Service<march_shared_msgs::srv::GetNodeJacobian>::SharedPtr m_service_node_jacobian;

};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_NODE_HPP_