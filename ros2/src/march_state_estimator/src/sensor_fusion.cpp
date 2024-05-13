/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/sensor_fusion.hpp"

#include <iostream>

SensorFusion::SensorFusion() {
    m_state.imu_position = Eigen::Vector3d::Zero();
    m_state.imu_velocity = Eigen::Vector3d::Zero();
    m_state.imu_orientation = Eigen::Quaterniond::Identity();
    m_state.left_foot_position = Eigen::Vector3d::Zero();
    m_state.right_foot_position = Eigen::Vector3d::Zero();
    m_state.accelerometer_bias = Eigen::Vector3d::Zero();
    m_state.gyroscope_bias = Eigen::Vector3d::Zero();
    m_state.left_foot_slippage = Eigen::Quaterniond::Identity();
    m_state.right_foot_slippage = Eigen::Quaterniond::Identity();
    m_state.covariance_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
}

const Eigen::Quaterniond SensorFusion::computeExponentialMap(const Eigen::Vector3d& vector) const {
    double angle = vector.norm();
    Eigen::Vector3d axis = vector.normalized();
    Eigen::Quaterniond quaternion;
    quaternion.w() = cos(0.5 * angle);
    quaternion.vec() = sin(0.5 * angle) * axis;
    quaternion.normalize();
    std::cout << "Quaternion: " << quaternion.w() << ", " << quaternion.vec().transpose() << std::endl;
    return quaternion;
}