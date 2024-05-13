/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

// #define DEBUG

#include "march_state_estimator/sensor_fusion.hpp"

#include <iostream>

SensorFusion::SensorFusion() {
    m_timestep = 0.05; // 20 Hz

    // Configure the initial state
    m_state.imu_position = Eigen::Vector3d(0.0, 0.0, 1.0295092403533455);
    m_state.imu_velocity = Eigen::Vector3d::Zero();
    m_state.imu_orientation = Eigen::Quaterniond::Identity();
    m_state.left_foot_position = Eigen::Vector3d(0.24743795173789007, 0.10789731603419578, 0.0);
    m_state.right_foot_position = Eigen::Vector3d(0.24743795173789007, -0.10789731603419578, 0.0);
    m_state.accelerometer_bias = Eigen::Vector3d::Zero();
    m_state.gyroscope_bias = Eigen::Vector3d::Zero();
    m_state.left_foot_slippage = Eigen::Quaterniond::Identity();
    m_state.right_foot_slippage = Eigen::Quaterniond::Identity();
    m_state.covariance_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE) * 1e-12;

    // Configure the initial observation
    m_observation.imu_acceleration = Eigen::Vector3d::Zero();
    m_observation.imu_angular_velocity = Eigen::Vector3d::Zero();
    m_observation.left_foot_position = Eigen::Vector3d::Zero();
    m_observation.right_foot_position = Eigen::Vector3d::Zero();
    m_observation.left_foot_slippage = Eigen::Quaterniond::Identity();
    m_observation.right_foot_slippage = Eigen::Quaterniond::Identity();

    // Configure the matrices
    m_dynamics_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    m_observation_matrix = Eigen::MatrixXd::Zero(MEASUREMENT_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    m_innovation_covariance_matrix = Eigen::MatrixXd::Identity(MEASUREMENT_DIMENSION_SIZE, MEASUREMENT_DIMENSION_SIZE);
    m_kalman_gain = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, MEASUREMENT_DIMENSION_SIZE);
}

void SensorFusion::predictState() {
    #ifdef DEBUG
    std::cout << "Computing prior state..." << std::endl;
    #endif
    // Compute the expected linear velocity
    Eigen::Vector3d expected_linear_velocity;
    expected_linear_velocity.noalias() = m_state.imu_velocity
        + m_timestep * (m_state.imu_orientation.toRotationMatrix().transpose() * computeMeasuredLinearAcceleration()); // + GRAVITY_VECTOR
    
    // Update the state with prior knowledge
    m_state.imu_position.noalias() += m_timestep * expected_linear_velocity;
    m_state.imu_velocity = expected_linear_velocity;
    m_state.imu_orientation = computeExponentialMap(m_timestep * computeMeasuredAngularVelocity()) * m_state.imu_orientation;
    m_state.covariance_matrix = computePriorCovarianceMatrix();

    #ifdef DEBUG
    std::cout << "Prior state computed." << std::endl;
    #endif
}

void SensorFusion::updateState() {
    #ifdef DEBUG
    std::cout << "Computing posterior state..." << std::endl;
    #endif

    const Eigen::VectorXd innovation = computeInnovation();
    #ifdef DEBUG
    std::cout << "Innovation: " << innovation.transpose() << std::endl;
    #endif

    computeInnovationCovarianceMatrix();
    #ifdef DEBUG
    std::cout << "Innovation covariance matrix computed." << std::endl;
    #endif

    computeKalmanGain();
    #ifdef DEBUG
    std::cout << "Kalman gain computed." << std::endl;
    #endif
    
    Eigen::VectorXd correction_vector;
    correction_vector.noalias() = m_kalman_gain * innovation;
    #ifdef DEBUG
    std::cout << "Correction vector computed." << std::endl;
    #endif

    // Update the state with the correction vector
    m_state.imu_position.noalias() += correction_vector.segment<3>(STATE_INDEX_POSITION);
    m_state.imu_velocity.noalias() += correction_vector.segment<3>(STATE_INDEX_VELOCITY);
    m_state.imu_orientation = computeExponentialMap(correction_vector.segment<3>(STATE_INDEX_ORIENTATION)) * m_state.imu_orientation;
    m_state.accelerometer_bias.noalias() += correction_vector.segment<3>(STATE_INDEX_ACCELEROMETER_BIAS);
    m_state.gyroscope_bias.noalias() += correction_vector.segment<3>(STATE_INDEX_GYROSCOPE_BIAS);
    m_state.left_foot_position.noalias() += correction_vector.segment<3>(STATE_INDEX_LEFT_FOOT_POSITION);
    m_state.right_foot_position.noalias() += correction_vector.segment<3>(STATE_INDEX_RIGHT_FOOT_POSITION);
    m_state.left_foot_slippage = computeExponentialMap(correction_vector.segment<3>(STATE_INDEX_LEFT_SLIPPAGE)) * m_state.left_foot_slippage;
    m_state.right_foot_slippage = computeExponentialMap(correction_vector.segment<3>(STATE_INDEX_RIGHT_SLIPPAGE)) * m_state.right_foot_slippage;
    m_state.covariance_matrix.noalias() -= m_kalman_gain * m_observation_matrix * m_state.covariance_matrix;


}

const Eigen::VectorXd SensorFusion::computeInnovation() const {
    Eigen::VectorXd innovation = Eigen::VectorXd::Zero(MEASUREMENT_DIMENSION_SIZE);
    Eigen::Matrix3d orientation_matrix = m_state.imu_orientation.toRotationMatrix();

    // Compute the innovation for the left and right foot positions
    innovation.segment<3>(MEASUREMENT_INDEX_LEFT_POSITION)
        = m_observation.left_foot_position - orientation_matrix * (m_state.left_foot_position - m_state.imu_position);
    innovation.segment<3>(MEASUREMENT_INDEX_RIGHT_POSITION)
        = m_observation.right_foot_position - orientation_matrix * (m_state.right_foot_position - m_state.imu_position);
    
    // Compute the innovation for the left and right foot slippage
    innovation.segment<3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE)
        = computeEulerAngles(m_observation.left_foot_slippage * m_state.imu_orientation.inverse());
    innovation.segment<3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE)
        = computeEulerAngles(m_observation.right_foot_slippage * m_state.imu_orientation.inverse());

    return innovation;
}

void SensorFusion::computeDynamicsMatrix() {
    m_dynamics_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    const Eigen::Matrix3d orientation_matrix = m_state.imu_orientation.toRotationMatrix().transpose();

    // Compute the dynamics matrix
    m_dynamics_matrix.block<3, 3>(STATE_INDEX_POSITION, STATE_INDEX_VELOCITY) = Eigen::Matrix3d::Identity() * m_timestep;
    m_dynamics_matrix.block<3, 3>(STATE_INDEX_VELOCITY, STATE_INDEX_ORIENTATION) 
        = -m_timestep * orientation_matrix * computeSkewSymmetricMatrix(computeMeasuredLinearAcceleration());
    m_dynamics_matrix.block<3, 3>(STATE_INDEX_ORIENTATION, STATE_INDEX_ORIENTATION)
        = -m_timestep * orientation_matrix * computeSkewSymmetricMatrix(computeMeasuredAngularVelocity());
    m_dynamics_matrix.block<3, 3>(STATE_INDEX_VELOCITY, STATE_INDEX_ACCELEROMETER_BIAS)
        = -m_timestep * orientation_matrix;
    m_dynamics_matrix.block<3, 3>(STATE_INDEX_ORIENTATION, STATE_INDEX_GYROSCOPE_BIAS)
        = -m_timestep * Eigen::Matrix3d::Identity();
}

void SensorFusion::computeObservationMatrix() {
    m_observation_matrix = Eigen::MatrixXd::Zero(MEASUREMENT_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    const Eigen::Matrix3d orientation_matrix = m_state.imu_orientation.toRotationMatrix();

    // Compute the observation components for the left and right foot positions
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, STATE_INDEX_POSITION) = -orientation_matrix;
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, STATE_INDEX_POSITION)= -orientation_matrix;
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, STATE_INDEX_ORIENTATION)
        = computeSkewSymmetricMatrix(orientation_matrix * (m_state.left_foot_position - m_state.imu_position));
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, STATE_INDEX_ORIENTATION)
        = computeSkewSymmetricMatrix(orientation_matrix * (m_state.right_foot_position - m_state.imu_position));
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, STATE_INDEX_LEFT_FOOT_POSITION) = orientation_matrix;
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, STATE_INDEX_RIGHT_FOOT_POSITION) = orientation_matrix;

    // Compute the observation components for the left and right foot slippage
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE, STATE_INDEX_ORIENTATION)
        = Eigen::Matrix3d::Identity();
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE, STATE_INDEX_ORIENTATION)
        = Eigen::Matrix3d::Identity();
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE, STATE_INDEX_LEFT_SLIPPAGE)
        = -1.0 * (m_state.imu_orientation * m_state.left_foot_slippage.inverse()).toRotationMatrix();
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE, STATE_INDEX_RIGHT_SLIPPAGE)
        = -1.0 * (m_state.imu_orientation * m_state.right_foot_slippage.inverse()).toRotationMatrix();
}

const Eigen::Quaterniond SensorFusion::computeExponentialMap(const Eigen::Vector3d& vector) const {
    double angle = vector.norm();
    Eigen::Vector3d axis = vector.normalized();
    Eigen::Quaterniond quaternion;
    quaternion.w() = cos(0.5 * angle);
    quaternion.vec() = sin(0.5 * angle) * axis;
    quaternion.normalize();
    return quaternion;
}

const Eigen::Matrix3d SensorFusion::computeSkewSymmetricMatrix(const Eigen::Vector3d& vector) const {
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix << 0.0, -vector[Z_AXIS], vector[Y_AXIS],
                             vector[Z_AXIS], 0.0, -vector[X_AXIS],
                             -vector[Y_AXIS], vector[X_AXIS], 0.0;
    return skew_symmetric_matrix;
}