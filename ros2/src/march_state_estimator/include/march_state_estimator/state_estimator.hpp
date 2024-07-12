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

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

class StateEstimator {
public:
    StateEstimator(const std::string& urdf_file_path);
    ~StateEstimator() = default;

    void configureStanceThresholds(const double& left_foot_threshold, const double& right_foot_threshold);

    void updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);
    void updateImuState(const sensor_msgs::msg::Imu::SharedPtr imu);
    void updateDynamicsState();

    inline Eigen::Vector3d getLeftFootForce() const { return m_left_foot_force; }
    inline Eigen::Vector3d getRightFootForce() const { return m_right_foot_force; }
    inline uint8_t getCurrentStanceLeg() const { return m_current_stance_leg; }
    uint8_t getNextStanceLeg(const double& left_foot_position, const double& right_foot_position) const;

    sensor_msgs::msg::JointState getEstimatedJointState() const;

    std::unordered_map<std::string, double> getJointPosition() const;
    Eigen::Quaterniond getInertialOrientation() const;
    Eigen::Vector3d getCOM() const;
    Eigen::Vector3d getCOMVelocity() const;
    geometry_msgs::msg::Point getZMP() const;
    std::vector<geometry_msgs::msg::Pose> getFootPoses() const;
    std::vector<double> getFootContactHeight() const;
    std::vector<double> getJointAcceleration(const std::vector<std::string>& joint_names) const;
    std::vector<double> getJointDynamicalTorques(const std::vector<std::string>& joint_names) const;
    std::vector<double> getJointExternalTorques(const std::vector<std::string>& joint_names) const;

    inline void setTimeStep(const double& timestep) { m_timestep = timestep; }

private:
    void configureRobotModel(const std::string& urdf_file_path);
    void configureJointState(const std::vector<std::string>& joint_names);
    void initializeCurrentStanceLeg();
    void initalizeFeetForce();

    Eigen::Vector3d computeTotalForce(const std::string& joint_name) const;
    Eigen::Vector3d computeExternalForce(const std::string& joint_name) const;

    void computeJointAcceleration(const Eigen::VectorXd& joint_velocities);
    void updateCurrentStanceLeg(
        const double& left_foot_torque, const double& right_foot_torque,
        const Eigen::Vector3d& left_foot_force, const Eigen::Vector3d& right_foot_force);

    std::vector<std::string> m_joint_names;

    Eigen::VectorXd m_joint_position;
    Eigen::VectorXd m_joint_velocity;
    Eigen::VectorXd m_joint_acceleration;

    Eigen::VectorXd m_joint_total_torque;
    Eigen::VectorXd m_joint_dynamical_torque;
    Eigen::VectorXd m_joint_external_torque;

    Eigen::Vector3d m_left_foot_force;
    Eigen::Vector3d m_right_foot_force;

    double m_left_foot_threshold;
    double m_right_foot_threshold;
    uint8_t m_current_stance_leg;

    Eigen::Quaterniond m_quaternion;
    double m_timestep;

    pinocchio::Model m_robot_model;
    std::unique_ptr<pinocchio::Data> m_robot_data;
};

#endif // MARCH_STATE_ESTIMATOR__STATE_ESTIMATOR_HPP_