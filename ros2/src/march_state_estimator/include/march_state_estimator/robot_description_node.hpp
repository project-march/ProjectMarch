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

    std::shared_ptr<RobotDescription> robot_description_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimatorVisualization>::SharedPtr node_positions_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TODO: Rename this to robot_descriptor_node
    // IMPLEMENTATION: persistent ROS2 services
    // IMPLEMENTATION: function to set up ROS2 services once and persistently connect
    // SOURCE: http://wiki.ros.org/roscpp/Overview/Services#Persistent_Connections
    // IMPLEMENTATION: function of ronnection logic in case of connection failures

    sensor_msgs::msg::JointState::SharedPtr m_joint_state_msg;

    rclcpp::Service<march_shared_msgs::srv::GetTaskReport>::SharedPtr m_service_task_report;

};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_NODE_HPP_