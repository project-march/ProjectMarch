/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_

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

#include "march_state_estimator/robot_description.hpp"
#include "march_state_estimator/torque_converter.hpp"

const unsigned int STATE_DIMENSION_SIZE = 27;
const unsigned int MEASUREMENT_DIMENSION_SIZE = 12;
const Eigen::Vector3d GRAVITY_VECTOR = Eigen::Vector3d(0.0, 0.0, -9.80665); // m/s^2

struct EKFState {
    Eigen::Vector3d imu_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond imu_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d left_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d right_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();
    Eigen::Quaterniond left_foot_slippage = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond right_foot_slippage = Eigen::Quaterniond::Identity();
    Eigen::MatrixXd covariance_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
};

struct EKFMeasurement {
    Eigen::Vector3d left_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d right_foot_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond left_foot_slippage = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond right_foot_slippage = Eigen::Quaterniond::Identity();
};

class SensorFusion {
public:
    SensorFusion(const RobotDescription::SharedPtr robot_description, const std::string& urdf_file_path);
    ~SensorFusion() = default;

    void configureJointNames(const std::vector<std::string>& joint_names);
    void configureStanceThresholds(const double& left_foot_threshold, const double& right_foot_threshold);

    void updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);
    void updateImuState(const sensor_msgs::msg::Imu::SharedPtr imu);
    void updateDynamicsState();
    void updateKalmanFilter();

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
    geometry_msgs::msg::Transform getRobotTransform() const;
    std::vector<geometry_msgs::msg::Pose> getFootPoses() const;
    std::vector<double> getFootContactHeight() const;
    std::vector<double> getJointAcceleration(const std::vector<std::string>& joint_names) const;
    std::vector<double> getJointDynamicalTorques(const std::vector<std::string>& joint_names) const;
    std::vector<double> getJointExternalTorques(const std::vector<std::string>& joint_names) const;

    std::vector<Eigen::Vector3d> getWorldTorqueInLegs() const;
    std::vector<Eigen::Vector3d> getWorldForceInLegs() const;

    Eigen::VectorXd getPosteriorStateVector() const;
    Eigen::Quaterniond getFilteredOrientation() const;
    Eigen::Quaterniond getExponentialMap(const Eigen::Vector3d& vector) const;
    Eigen::Matrix3d getSkewSymmetricMatrix(const Eigen::Vector3d& vector) const;
    Eigen::MatrixXd getStateTransitionMatrix(const EKFState& state) const;
    Eigen::MatrixXd getObservationModelMatrix(const EKFState& state) const;
    Eigen::MatrixXd getNoiseJacobianMatrix(const EKFState& state) const;

    void setTimeStep(const double& timestep);
    void setProcessNoiseAccelerationVector(const Eigen::Vector3d& process_noise_acceleration);
    void setProcessNoiseAngularVelocityVector(const Eigen::Vector3d& process_noise_angular_velocity);
    void setProcessNoiseCovarianceMatrix(
        const Eigen::Vector3d& process_noise_velocity,
        const Eigen::Vector3d& process_noise_acceleration,
        const Eigen::Vector3d& process_noise_angular_velocity, 
        const Eigen::Vector3d& process_noise_feet_position, 
        const Eigen::Vector3d& process_noise_accelerometer_bias, 
        const Eigen::Vector3d& process_noise_gyroscope_bias, 
        const Eigen::Vector3d& process_noise_feet_slippage);
    void setMeasurementNoiseCovarianceMatrix(
        const Eigen::Vector3d& measurement_noise_feet_position,
        const Eigen::Vector3d& measurement_noise_feet_slippage);
    void updateProcessNoiseCovarianceMatrix(const EKFState& state_posterior);

    Eigen::Vector3d calculateExpectedMeasuredAcceleration(const Eigen::Vector3d& accelerometer_bias) const;
    Eigen::Vector3d calculateExpectedMeasuredAngularVelocity(const Eigen::Vector3d& gyroscope_bias) const;
    Eigen::VectorXd calculateInnovation(const EKFState& state_priori) const;
    Eigen::VectorXd calculateStateCorrectionVector(const Eigen::MatrixXd& kalman_gain, const Eigen::VectorXd& innovation) const;
    Eigen::MatrixXd calculateInnovationCovarianceMatrix(const EKFState& state_priori) const;
    Eigen::MatrixXd calculateKalmanGain(const EKFState& state_priori, const Eigen::MatrixXd& observation_model_matrix) const;
    Eigen::MatrixXd calculatePriorCovarianceMatrix(const EKFState& state_posterior) const;
    Eigen::MatrixXd calculateEstimatedCovarianceMatrix(const EKFState& state_priori, 
        const Eigen::MatrixXd& kalman_gain, const Eigen::MatrixXd& observation_model) const;
    EKFState calculatePriorState(const EKFState& state_posterior) const;
    EKFState calculatePosteriorState(const EKFState& state_priori) const;

    Eigen::VectorXd getEKFStateVector(const EKFState& state) const;

    bool hasConverged(const Eigen::Quaterniond& desired_orientation, const double& threshold) const;
    sensor_msgs::msg::Imu::SharedPtr getFilteredImuMsg() const;
    geometry_msgs::msg::Vector3::SharedPtr getFilteredOrientationMsg() const;

private:
    void updateCurrentStanceLeg(
        const double& left_foot_torque, const double& right_foot_torque,
        const Eigen::Vector3d& left_foot_force, const Eigen::Vector3d& right_foot_force);

    inline Eigen::Vector3d msgToEigenVector3d(const geometry_msgs::msg::Vector3& vector) const;
    inline Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& quaternion) const;
    inline Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d& rotation_matrix) const;

    RobotDescription::SharedPtr m_robot_description;
    TorqueConverter::UniquePtr m_torque_converter;
    sensor_msgs::msg::JointState::SharedPtr m_recent_joint_state_msg;
    sensor_msgs::msg::Imu::SharedPtr m_recent_imu_msg;
    double m_state_estimator_timestep;

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

    double m_timestep;
    EKFState m_state_posterior;
    Eigen::Vector3d m_process_noise_acceleration;
    Eigen::Vector3d m_process_noise_angular_velocity;
    Eigen::MatrixXd m_process_noise_covariance_matrix;
    Eigen::MatrixXd m_measurement_noise_covariance_matrix;
};

#endif // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_