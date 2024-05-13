/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_

#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#define STATE_DIMENSION_SIZE            27
#define STATE_INDEX_POSITION            0
#define STATE_INDEX_VELOCITY            3
#define STATE_INDEX_ORIENTATION         6
#define STATE_INDEX_LEFT_FOOT_POSITION  9
#define STATE_INDEX_RIGHT_FOOT_POSITION 12
#define STATE_INDEX_ACCELEROMETER_BIAS  15
#define STATE_INDEX_GYROSCOPE_BIAS      18
#define STATE_INDEX_LEFT_SLIPPAGE       21
#define STATE_INDEX_RIGHT_SLIPPAGE      24

#define MEASUREMENT_DIMENSION_SIZE  12
#define MEASUREMENT_INDEX_LEFT_POSITION    0
#define MEASUREMENT_INDEX_RIGHT_POSITION   3
#define MEASUREMENT_INDEX_LEFT_SLIPPAGE    6
#define MEASUREMENT_INDEX_RIGHT_SLIPPAGE   9

#define ROTATION_ROLL  0
#define ROTATION_PITCH 1
#define ROTATION_YAW   2

#define QUATERNION_W 0
#define QUATERNION_X 1
#define QUATERNION_Y 2
#define QUATERNION_Z 3

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

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
    Eigen::Vector3d imu_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_angular_velocity = Eigen::Vector3d::Zero();
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

    inline void estimateState() {
        predictState();
        updateState();
    }
    inline const EKFState& getState() const { return m_state; }
    inline void setObservation(const EKFObservation& observation) { m_observation = observation; }
    inline void setTimestep(const double& timestep) { m_timestep = timestep; }

private:
    void predictState();
    void updateState();

    inline const Eigen::Vector3d computeMeasuredLinearAcceleration() const {
        return m_observation.imu_acceleration - m_state.accelerometer_bias;
    }
    inline const Eigen::Vector3d computeMeasuredAngularVelocity() const {
        return m_observation.imu_angular_velocity - m_state.gyroscope_bias;
    }
    inline const Eigen::MatrixXd computePriorCovarianceMatrix() const {
        return m_dynamics_matrix * m_state.covariance_matrix * m_dynamics_matrix.transpose();
    }
    inline void computeInnovationCovarianceMatrix() {
        m_innovation_covariance_matrix = m_observation_matrix * m_state.covariance_matrix * m_observation_matrix.transpose();
    }
    inline void computeKalmanGain() {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_innovation_covariance_matrix(m_innovation_covariance_matrix);
        cod_innovation_covariance_matrix.setThreshold(1e-6); // TODO: Set a threshold for the SVD decomposition
        m_kalman_gain = m_state.covariance_matrix * m_observation_matrix.transpose() * cod_innovation_covariance_matrix.pseudoInverse();
    }
    const Eigen::VectorXd computeInnovation() const;
    void computeDynamicsMatrix();
    void computeObservationMatrix();

    inline const Eigen::Vector3d computeEulerAngles(const Eigen::Quaterniond& orientation) const {
        return orientation.toRotationMatrix().eulerAngles(ROTATION_ROLL, ROTATION_PITCH, ROTATION_YAW);
    }
    const Eigen::Quaterniond computeExponentialMap(const Eigen::Vector3d& vector) const;
    const Eigen::Matrix3d computeSkewSymmetricMatrix(const Eigen::Vector3d& vector) const;

    double m_timestep;

    EKFState m_state;
    EKFObservation m_observation;
    Eigen::MatrixXd m_dynamics_matrix;
    Eigen::MatrixXd m_observation_matrix;
    Eigen::MatrixXd m_innovation_covariance_matrix;
    Eigen::MatrixXd m_kalman_gain;

    friend class SensorFusionTest;
};

#endif  // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_