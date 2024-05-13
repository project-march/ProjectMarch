/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_HPP_
#define MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_HPP_

#define EULER_ROLL_AXIS     0
#define EULER_PITCH_AXIS    1
#define EULER_YAW_AXIS      2

#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "march_state_estimator/robot_description/robot_description.hpp"
#include "march_state_estimator/torque_converter.hpp"

class StateEstimator {
public:
    StateEstimator(const RobotDescription::SharedPtr robot_description, const std::string& urdf_file_path);
    ~StateEstimator() = default;

    void configureJointNames(const std::vector<std::string>& joint_names);
    void configureStanceThresholds(const double& left_foot_threshold, const double& right_foot_threshold);

    void updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);
    void updateImuState(const sensor_msgs::msg::Imu::SharedPtr imu);
    void updateDynamicsState();

    Eigen::Vector3d getLeftFootForce() const;
    Eigen::Vector3d getRightFootForce() const;
    uint8_t getCurrentStanceLeg() const;
    uint8_t getNextStanceLeg(const double& left_foot_position, const double& right_foot_position) const;

    sensor_msgs::msg::JointState getEstimatedJointState() const;

    RobotNode::JointNameToValueMap getJointPositions() const;
    Eigen::Quaterniond getInertialOrientation() const;
    Eigen::Vector3d getCOM() const;
    Eigen::Vector3d getCOMVelocity() const;
    geometry_msgs::msg::Point getZMP() const;
    std::vector<geometry_msgs::msg::Pose> getFootPoses() const;
    std::vector<double> getFootContactHeight() const;
    std::vector<double> getJointAcceleration(const std::vector<std::string>& joint_names) const;
    std::vector<double> getJointDynamicalTorques(const std::vector<std::string>& joint_names) const;
    std::vector<double> getJointExternalTorques(const std::vector<std::string>& joint_names) const;

    std::vector<Eigen::Vector3d> getWorldTorqueInLegs() const;
    std::vector<Eigen::Vector3d> getWorldForceInLegs() const;

    void setTimeStep(const double& timestep);

private:
    void updateCurrentStanceLeg(
        const double& left_foot_torque, const double& right_foot_torque,
        const Eigen::Vector3d& left_foot_force, const Eigen::Vector3d& right_foot_force);

    RobotDescription::SharedPtr m_robot_description;
    TorqueConverter::UniquePtr m_torque_converter;
    sensor_msgs::msg::JointState::SharedPtr m_recent_joint_state_msg;
    sensor_msgs::msg::Imu::SharedPtr m_recent_imu_msg;

    std::vector<std::string> m_joint_names;
    RobotNode::JointNameToValueMap m_joint_positions;
    RobotNode::JointNameToValueMap m_joint_velocities;
    RobotNode::JointNameToValueMap m_joint_accelerations;
    RobotNode::JointNameToValueMap m_joint_total_torques;
    RobotNode::JointNameToValueMap m_joint_dynamical_torques;
    RobotNode::JointNameToValueMap m_joint_external_torques;
    Eigen::Vector3d m_left_foot_force;
    Eigen::Vector3d m_right_foot_force;

    double m_left_foot_threshold;
    double m_right_foot_threshold;
    uint8_t m_current_stance_leg;

    Eigen::Quaterniond m_quaternion;
    double m_state_estimator_timestep;
};

#endif // MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_HPP_