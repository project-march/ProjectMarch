/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_

#pragma once
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#define STATE_DIMENSION_SIZE        27
#define MEASUREMENT_DIMENSION_SIZE  12

#define ROTATION_ROLL  0
#define ROTATION_PITCH 1
#define ROTATION_YAW   2

const Eigen::Vector3d GRAVITY_VECTOR = Eigen::Vector3d(0.0, 0.0, -9.81); // m/s^2

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

struct EKFObservation {
    Eigen::Vector3d left_foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d right_foot_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond left_foot_slippage = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond right_foot_slippage = Eigen::Quaterniond::Identity();
};

// Forward declaration of SensorFusionTest
class SensorFusionTest;

class SensorFusion {
public:
    SensorFusion();
    ~SensorFusion() = default;

    inline const EKFState& getState() const { return m_state; }
    inline void setMeasurement(const EKFObservation& observation) { m_observation = observation; }

private:
    inline const Eigen::Quaterniond computeOrientation(const Eigen::Vector4d& quaternion) {
        return Eigen::Quaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    }
    inline const Eigen::Vector3d computeEulerAngles(const Eigen::Quaterniond& orientation) {
        return orientation.toRotationMatrix().eulerAngles(ROTATION_ROLL, ROTATION_PITCH, ROTATION_YAW);
    }
    const Eigen::Quaterniond computeExponentialMap(const Eigen::Vector3d& vector) const;

    EKFState m_state;
    EKFObservation m_observation;

    friend class SensorFusionTest;
};

#endif  // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_