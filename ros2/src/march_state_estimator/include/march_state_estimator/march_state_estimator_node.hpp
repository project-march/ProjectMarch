#ifndef MARCH_STATE_ESTIMATOR__MARCH_STATE_ESTIMATOR_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__MARCH_STATE_ESTIMATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "march_shared_msgs/msg/state_estimator_visualization.hpp"
#include "march_state_estimator/robot_description.hpp"


class MarchStateEstimatorNode : public rclcpp::Node
{
public:
    MarchStateEstimatorNode();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishNodePositions();

    std::shared_ptr<RobotDescription> robot_description_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<march_shared_msgs::msg::StateEstimatorVisualization>::SharedPtr node_positions_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

#endif // MARCH_STATE_ESTIMATOR__MARCH_STATE_ESTIMATOR_NODE_HPP_