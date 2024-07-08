/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/sensor_fusion.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <cmath>

SensorFusion::SensorFusion(double timestep) {
    m_timestep = timestep;

    // Configure robot parameters
    std::string urdf_file_path = ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march9/march9.urdf";
    pinocchio::urdf::buildModel(urdf_file_path, m_robot_model);
    m_robot_data = std::make_unique<pinocchio::Data>(m_robot_model);
    m_joint_position = Eigen::VectorXd::Zero(m_robot_model.nv);

    // Set joint index according to the URDF assuming there are 2 legs
    int num_joints_per_leg = m_robot_model.nv >> 1;
    for (int i = 0; i < NUM_OF_LEGS; i++) {
        m_joint_idx[i] = num_joints_per_leg * (i + 1);
    }

    // Configure the initial state
    m_state.imu_position = Eigen::Vector3d(0.0, 0.0, 1.0295092403533455);
    m_state.imu_velocity = Eigen::Vector3d::Zero();
    m_state.imu_orientation = Eigen::Quaterniond::Identity();
    m_state.left_foot_position = Eigen::Vector3d(0.24919716760324423, 0.18988430009579504, 0.0);
    m_state.right_foot_position = Eigen::Vector3d(0.24919716760324423, -0.18988430009579504, 0.0);
    m_state.accelerometer_bias = Eigen::Vector3d::Zero();
    m_state.gyroscope_bias = Eigen::Vector3d::Zero();
    m_state.left_foot_slippage = Eigen::Quaterniond::Identity();
    m_state.right_foot_slippage = Eigen::Quaterniond::Identity();
    m_state.covariance_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);

    // Configure the initial observation
    m_observation.joint_position = Eigen::VectorXd::Zero(m_robot_model.nv);
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
    m_process_noise_covariance_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    m_observation_noise_covariance_matrix = Eigen::MatrixXd::Identity(MEASUREMENT_DIMENSION_SIZE, MEASUREMENT_DIMENSION_SIZE);
    m_observation_noise_covariance_joint_matrix = Eigen::MatrixXd::Identity(m_robot_model.nv, m_robot_model.nv);
    m_observation_noise_covariance_position_matrix = Eigen::MatrixXd::Identity(CARTESIAN_DIMENSION_SIZE, CARTESIAN_DIMENSION_SIZE);
    m_observation_noise_covariance_slippage_matrix = Eigen::MatrixXd::Identity(CARTESIAN_DIMENSION_SIZE, CARTESIAN_DIMENSION_SIZE);

    // Initialize the performance cost with a large value
    m_performance_cost = 1e23;
}

void SensorFusion::predictState() {
    #ifdef DEBUG
    std::cout << "Computing prior state..." << std::endl;
    #endif
    // Compute the expected linear velocity
    Eigen::Vector3d expected_linear_velocity;
    expected_linear_velocity.noalias() = m_timestep * (m_state.imu_orientation.toRotationMatrix().transpose() * computeMeasuredLinearAcceleration() + GRAVITY_VECTOR);
    
    // Update the state with prior knowledge
    m_state.imu_position.noalias() += m_timestep * m_state.imu_velocity + 0.5 * m_timestep * expected_linear_velocity;
    m_state.imu_velocity.noalias() += expected_linear_velocity;
    m_state.imu_orientation = computeExponentialMap(m_timestep * computeMeasuredAngularVelocity()) * m_state.imu_orientation;

    // Normalize the orientations
    m_state.imu_orientation.normalize();

    // Update the covariance matrix with prior knowledge
    computeDynamicsMatrix();
    computeProcessNoiseCovarianceMatrix();
    m_state.covariance_matrix = computePriorCovarianceMatrix();

    #ifdef DEBUG
    Eigen::VectorXd prior_state(STATE_DIMENSION_SIZE);
    prior_state.segment<3>(STATE_INDEX_POSITION) = m_state.imu_position;
    prior_state.segment<3>(STATE_INDEX_VELOCITY) = m_state.imu_velocity;
    prior_state.segment<3>(STATE_INDEX_ORIENTATION) = computeEulerAngles(m_state.imu_orientation);
    prior_state.segment<3>(STATE_INDEX_ACCELEROMETER_BIAS) = m_state.accelerometer_bias;
    prior_state.segment<3>(STATE_INDEX_GYROSCOPE_BIAS) = m_state.gyroscope_bias;
    prior_state.segment<3>(STATE_INDEX_LEFT_FOOT_POSITION) = m_state.left_foot_position;
    prior_state.segment<3>(STATE_INDEX_RIGHT_FOOT_POSITION) = m_state.right_foot_position;
    prior_state.segment<3>(STATE_INDEX_LEFT_SLIPPAGE) = computeEulerAngles(m_state.left_foot_slippage);
    prior_state.segment<3>(STATE_INDEX_RIGHT_SLIPPAGE) = computeEulerAngles(m_state.right_foot_slippage);
    std::cout << "Prior state: " << prior_state.transpose() << std::endl;
    #endif
}

void SensorFusion::updateState() {
    #ifdef DEBUG
    std::cout << "Computing posterior state..." << std::endl;
    #endif

    const Eigen::VectorXd innovation = computeInnovation();
    computeObservationMatrix();
    computeObservationNoiseCovarianceMatrix();
    computeInnovationCovarianceMatrix();
    computeKalmanGain();

    // Compute the correction vector to update the state
    Eigen::VectorXd correction_vector;
    correction_vector.noalias() = m_kalman_gain * innovation;
    #ifdef DEBUG
    std::cout << "Correction vector: " << correction_vector.transpose() << std::endl;
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
    m_state.covariance_matrix.noalias() = computePosteriorCovarianceMatrix();

    // Normalize the orientations
    m_state.imu_orientation.normalize();
    m_state.left_foot_slippage.normalize();
    m_state.right_foot_slippage.normalize();

    // Compute the performance cost to evaluate and optimize the sensor fusion algorithm
    computePerformanceCost(innovation);

    #ifdef DEBUG
    Eigen::VectorXd posterior_state(STATE_DIMENSION_SIZE);
    posterior_state.segment<3>(STATE_INDEX_POSITION) = m_state.imu_position;
    posterior_state.segment<3>(STATE_INDEX_VELOCITY) = m_state.imu_velocity;
    posterior_state.segment<3>(STATE_INDEX_ORIENTATION) = computeEulerAngles(m_state.imu_orientation);
    posterior_state.segment<3>(STATE_INDEX_ACCELEROMETER_BIAS) = m_state.accelerometer_bias;
    posterior_state.segment<3>(STATE_INDEX_GYROSCOPE_BIAS) = m_state.gyroscope_bias;
    posterior_state.segment<3>(STATE_INDEX_LEFT_FOOT_POSITION) = m_state.left_foot_position;
    posterior_state.segment<3>(STATE_INDEX_RIGHT_FOOT_POSITION) = m_state.right_foot_position;
    posterior_state.segment<3>(STATE_INDEX_LEFT_SLIPPAGE) = computeEulerAngles(m_state.left_foot_slippage);
    posterior_state.segment<3>(STATE_INDEX_RIGHT_SLIPPAGE) = computeEulerAngles(m_state.right_foot_slippage);
    std::cout << "Posterior state: " << posterior_state.transpose() << std::endl;
    #endif
}

const Eigen::VectorXd SensorFusion::computeInnovation() const {
    Eigen::VectorXd innovation = Eigen::VectorXd::Zero(MEASUREMENT_DIMENSION_SIZE);
    Eigen::Matrix3d orientation_matrix = m_state.imu_orientation.toRotationMatrix();

    // Compute the innovation for the left and right foot positions
    innovation.segment<3>(MEASUREMENT_INDEX_LEFT_POSITION).noalias()
        = m_observation.left_foot_position - orientation_matrix * (m_state.left_foot_position - m_state.imu_position);
    innovation.segment<3>(MEASUREMENT_INDEX_RIGHT_POSITION).noalias()
        = m_observation.right_foot_position - orientation_matrix * (m_state.right_foot_position - m_state.imu_position);
    
    // Compute the innovation for the left and right foot slippage
    // innovation.segment<3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE).noalias()
    //     = computeEulerAngles(m_observation.left_foot_slippage * m_state.imu_orientation.inverse());
    // innovation.segment<3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE).noalias()
    //     = computeEulerAngles(m_observation.right_foot_slippage * m_state.imu_orientation.inverse());

    #ifdef DEBUG
    std::cout << "Innovation: " << innovation.transpose() << std::endl;
    #endif

    return innovation;
}

const Eigen::MatrixXd SensorFusion::computeNoiseJacobianMatrix() const {
    Eigen::MatrixXd noise_jacobian_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    Eigen::Matrix3d orientation_matrix = m_state.imu_orientation.toRotationMatrix().transpose();
    noise_jacobian_matrix.block<3,3>(STATE_INDEX_POSITION, STATE_INDEX_POSITION) = -orientation_matrix;
    noise_jacobian_matrix.block<3,3>(STATE_INDEX_VELOCITY, STATE_INDEX_VELOCITY) = -orientation_matrix;
    noise_jacobian_matrix.block<3,3>(STATE_INDEX_LEFT_FOOT_POSITION, STATE_INDEX_LEFT_FOOT_POSITION) = orientation_matrix;
    noise_jacobian_matrix.block<3,3>(STATE_INDEX_RIGHT_FOOT_POSITION, STATE_INDEX_RIGHT_FOOT_POSITION) = orientation_matrix;
    #ifdef DEBUG
    std::cout << "Noise Jacobian matrix:\n" << noise_jacobian_matrix << std::endl;
    #endif
    return noise_jacobian_matrix;
}

void SensorFusion::computeDynamicsMatrix() {
    m_dynamics_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    const Eigen::Matrix3d orientation_matrix = m_state.imu_orientation.toRotationMatrix().transpose();

    // Compute the dynamics matrix
    m_dynamics_matrix.block<3,3>(STATE_INDEX_POSITION, STATE_INDEX_VELOCITY) = Eigen::Matrix3d::Identity() * m_timestep;
    m_dynamics_matrix.block<3,3>(STATE_INDEX_VELOCITY, STATE_INDEX_ORIENTATION) 
        = -m_timestep * orientation_matrix * computeSkewSymmetricMatrix(computeMeasuredLinearAcceleration());
    m_dynamics_matrix.block<3,3>(STATE_INDEX_ORIENTATION, STATE_INDEX_ORIENTATION)
        -= -m_timestep * computeSkewSymmetricMatrix(computeMeasuredAngularVelocity());
    m_dynamics_matrix.block<3,3>(STATE_INDEX_VELOCITY, STATE_INDEX_ACCELEROMETER_BIAS)
        = -m_timestep * orientation_matrix;
    m_dynamics_matrix.block<3,3>(STATE_INDEX_ORIENTATION, STATE_INDEX_GYROSCOPE_BIAS)
        = -m_timestep * Eigen::Matrix3d::Identity();

    #ifdef DEBUG
    std::cout << "Dynamics matrix:\n" << m_dynamics_matrix << std::endl;
    #endif
}

void SensorFusion::computeObservationMatrix() {
    m_observation_matrix = Eigen::MatrixXd::Zero(MEASUREMENT_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    const Eigen::Matrix3d orientation_matrix = m_state.imu_orientation.toRotationMatrix();

    // Compute the observation components for the left and right foot positions
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, STATE_INDEX_POSITION) = -orientation_matrix;
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, STATE_INDEX_POSITION)= -orientation_matrix;
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, STATE_INDEX_ORIENTATION).noalias()
        = computeSkewSymmetricMatrix(orientation_matrix * (m_state.left_foot_position - m_state.imu_position));
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, STATE_INDEX_ORIENTATION).noalias()
        = computeSkewSymmetricMatrix(orientation_matrix * (m_state.right_foot_position - m_state.imu_position));
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, STATE_INDEX_LEFT_FOOT_POSITION) = orientation_matrix;
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, STATE_INDEX_RIGHT_FOOT_POSITION) = orientation_matrix;

    // Compute the observation components for the left and right foot slippage
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE, STATE_INDEX_ORIENTATION)
        = Eigen::Matrix3d::Identity();
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE, STATE_INDEX_ORIENTATION)
        = Eigen::Matrix3d::Identity();
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE, STATE_INDEX_LEFT_SLIPPAGE).noalias()
        = -1.0 * (m_state.imu_orientation * m_state.left_foot_slippage.inverse()).toRotationMatrix();
    m_observation_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE, STATE_INDEX_RIGHT_SLIPPAGE).noalias()
        = -1.0 * (m_state.imu_orientation * m_state.right_foot_slippage.inverse()).toRotationMatrix();

    #ifdef DEBUG
    std::cout << "Observation matrix:\n" << m_observation_matrix << std::endl;
    #endif
}

void SensorFusion::computeObservationNoiseCovarianceMatrix() {
    // Compute the Jacobians per leg
    Eigen::MatrixXd jacobians[NUM_OF_LEGS];
    for (int i = 0; i < NUM_OF_LEGS; i++) {
        // Fixed-size template cannot be used with aliases
        jacobians[i] = Eigen::MatrixXd::Zero(SE3_DIMENSION_SIZE, m_robot_model.nv);
        pinocchio::computeJointJacobian(m_robot_model, *m_robot_data, m_joint_position, m_joint_idx[i], jacobians[i]);
        // #ifdef DEBUG
        // std::cout << "Jacobian " << i << ":\n" << jacobians[i] << std::endl;
        // #endif
    }
    m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, MEASUREMENT_INDEX_LEFT_POSITION).noalias()
        = m_observation_noise_covariance_left_position_matrix + jacobians[LEFT_LEG_INDEX].block(JACOBIAN_POSITION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv)
            * m_observation_noise_covariance_joint_matrix * jacobians[LEFT_LEG_INDEX].block(JACOBIAN_POSITION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv).transpose();
    m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, MEASUREMENT_INDEX_RIGHT_POSITION).noalias()
        = m_observation_noise_covariance_right_position_matrix + jacobians[RIGHT_LEG_INDEX].block(JACOBIAN_POSITION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv)
            * m_observation_noise_covariance_joint_matrix * jacobians[RIGHT_LEG_INDEX].block(JACOBIAN_POSITION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv).transpose();
    m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE, MEASUREMENT_INDEX_LEFT_SLIPPAGE).noalias()
        = m_observation_noise_covariance_left_slippage_matrix + jacobians[LEFT_LEG_INDEX].block(JACOBIAN_ORIENTATION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv)
            * m_observation_noise_covariance_joint_matrix * jacobians[LEFT_LEG_INDEX].block(JACOBIAN_ORIENTATION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv).transpose();
    m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE, MEASUREMENT_INDEX_RIGHT_SLIPPAGE).noalias()
        = m_observation_noise_covariance_right_slippage_matrix + jacobians[RIGHT_LEG_INDEX].block(JACOBIAN_ORIENTATION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv)
            * m_observation_noise_covariance_joint_matrix * jacobians[RIGHT_LEG_INDEX].block(JACOBIAN_ORIENTATION_INDEX, 0, CARTESIAN_DIMENSION_SIZE, m_robot_model.nv).transpose();
    m_observation_noise_covariance_matrix.noalias() = m_observation_noise_covariance_matrix / m_timestep;
    #ifdef DEBUG
    std::cout << "Observation noise covariance matrix:\n" << m_observation_noise_covariance_matrix << std::endl;
    #endif
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

    // #ifdef DEBUG
    // std::cout << "Skew-symmetric matrix:\n" << skew_symmetric_matrix << std::endl;
    // #endif

    return skew_symmetric_matrix;
}