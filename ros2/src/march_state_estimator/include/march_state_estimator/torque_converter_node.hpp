/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_NODE_HPP_
#define MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "march_state_estimator/torque_converter.hpp"
#include "march_state_estimator/robot_description.hpp"

#include "march_shared_msgs/msg/joint_efforts.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class TorqueConverterNode : public rclcpp::Node
{
public:
  TorqueConverterNode();
  ~TorqueConverterNode() = default;

private:
    void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
    void desiredJointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void publishDesiredJointEfforts();

    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_sub;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_desired_joint_trajectory_sub;
    rclcpp::Publisher<march_shared_msgs::msg::JointEfforts>::SharedPtr m_joint_efforts_pub;

    std::vector<std::string> m_joint_names;
    RobotNode::JointNameToValueMap m_joint_positions;
    RobotNode::JointNameToValueMap m_joint_velocities;
    RobotNode::JointNameToValueMap m_joint_accelerations;
    RobotNode::JointNameToValueMap m_joint_external_torques;

    RobotDescription::SharedPtr m_robot_description;
    TorqueConverter::UniquePtr m_torque_converter;
};

#endif // MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_NODE_HPP_