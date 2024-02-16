/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "march_state_estimator/robot_description.hpp"
#include "march_state_estimator/torque_converter.hpp"

class SensorFusion {
public:
    SensorFusion(const RobotDescription::SharedPtr robot_description);
    ~SensorFusion() = default;

    void configureJointNames(const std::vector<std::string>& joint_names);
    void updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);
    void updateQuaternionValues(const geometry_msgs::msg::Quaternion* quaternion);
    uint8_t updateStanceLeg(
        const geometry_msgs::msg::Point* left_foot_position, const geometry_msgs::msg::Point* right_foot_position);

    Eigen::Vector3d getCOM() const;
    geometry_msgs::msg::Point getZMP() const;
    std::vector<geometry_msgs::msg::Pose> getFootPoses() const;

private:
    RobotDescription::SharedPtr m_robot_description;
    TorqueConverter::UniquePtr m_torque_converter;

    RobotNode::JointNameToValueMap m_joint_positions;
    RobotNode::JointNameToValueMap m_joint_velocities;
    RobotNode::JointNameToValueMap m_joint_accelerations;
    RobotNode::JointNameToValueMap m_joint_total_torques;

    Eigen::Quaterniond m_quaternion;
};

#endif // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_