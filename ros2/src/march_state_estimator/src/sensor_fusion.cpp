/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/sensor_fusion.hpp"

SensorFusion::SensorFusion(const RobotDescription::SharedPtr robot_description)
{
    m_robot_description = robot_description;
    m_torque_converter = std::make_unique<TorqueConverter>(robot_description);

    m_state_posterior = EKFState();

    // TODO: get from parameter server
    m_state_posterior.imu_position = Eigen::Vector3d(0.0, 0.0, 0.77);
    m_state_posterior.left_foot_position = Eigen::Vector3d(0.38, 0.037, 0.0);
    m_state_posterior.right_foot_position = Eigen::Vector3d(0.38, -0.049, 0.0);
    m_timestep = 1e-15;

    // m_process_noise_acceleration = Eigen::Vector3d(2.16e-5, 2.16e-5, 1.94e-5);
    // m_process_noise_angular_velocity = Eigen::Vector3d(0.118174, 0.118174, 0.118174);
    // setProcessNoiseCovarianceMatrix(
    //     m_process_noise_acceleration, m_process_noise_acceleration, m_process_noise_angular_velocity,
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23), // feet position
    //     Eigen::Vector3d(3e-7, 3e-7, 3e-7), // accelerometer bias
    //     Eigen::Vector3d(1.454e-5, 1.454e-5, 1.454e-5), // gyroscope bias
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23) // feet slippage
    // );
    // setMeasurementNoiseCovarianceMatrix(
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23), // feet position
    //     Eigen::Vector3d(1e-23, 1e-23, 1e-23) // feet slippage
    // );

    m_process_noise_acceleration = Eigen::Vector3d::Zero();
    m_process_noise_angular_velocity = Eigen::Vector3d::Zero();
    setProcessNoiseCovarianceMatrix(
        m_process_noise_acceleration, m_process_noise_acceleration, m_process_noise_angular_velocity,
        Eigen::Vector3d::Zero(), // feet position
        Eigen::Vector3d::Zero(), // accelerometer bias
        Eigen::Vector3d::Zero(), // gyroscope bias
        Eigen::Vector3d::Zero() // feet slippage
    );
    setMeasurementNoiseCovarianceMatrix(
        Eigen::Vector3d::Zero(), // feet position
        Eigen::Vector3d::Zero() // feet slippage
    );
}

void SensorFusion::configureJointNames(const std::vector<std::string>& joint_names)
{
    m_joint_positions.clear();
    m_joint_velocities.clear();
    m_joint_accelerations.clear();
    m_joint_total_torques.clear();

    for (const auto& joint_name : joint_names) {
        m_joint_positions[joint_name] = 0.0;
        m_joint_velocities[joint_name] = 0.0;
        m_joint_accelerations[joint_name] = 0.0;
        m_joint_total_torques[joint_name] = 0.0;
    }
}

void SensorFusion::updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    for (unsigned int i = 0; i < joint_state->name.size(); i++) {
        m_joint_positions[joint_state->name[i]] = joint_state->position[i];
        m_joint_velocities[joint_state->name[i]] = joint_state->velocity[i];
        m_joint_total_torques[joint_state->name[i]] = joint_state->effort[i];
    }
    m_joint_accelerations
        = m_torque_converter->getDynamicalJointAccelerations(m_joint_positions, m_joint_total_torques);
}

void SensorFusion::updateImu(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    m_recent_imu_msg = imu;
    m_quaternion = Eigen::Quaterniond(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    m_robot_description->setInertialOrientation(m_quaternion);
}

uint8_t SensorFusion::updateStanceLeg(
    const geometry_msgs::msg::Point* left_foot_position, const geometry_msgs::msg::Point* right_foot_position)
{
    uint8_t stance_leg = 0b11;
    const double margin = 0.01;

    // TODO: Optimize this function.
    if (abs(left_foot_position->x - right_foot_position->x) <= margin) {
        stance_leg = 0b11;
    } else if (left_foot_position->x + margin <= right_foot_position->x) {
        stance_leg = 0b10;
    } else if (left_foot_position->x - margin > right_foot_position->x) {
        stance_leg = 0b01;
    }

    m_robot_description->setStanceLeg(stance_leg,
        Eigen::Vector3d(left_foot_position->x, left_foot_position->y, left_foot_position->z),
        Eigen::Vector3d(right_foot_position->x, right_foot_position->y, right_foot_position->z));
    return stance_leg;
}

void SensorFusion::updateKalmanFilter()
{
    updateProcessNoiseCovarianceMatrix(m_state_posterior);
    EKFState state_prior = calculatePriorState(m_state_posterior);
    m_state_posterior = calculatePosteriorState(state_prior);
}

Eigen::Vector3d SensorFusion::getCOM() const
{
    Eigen::Vector3d com_body_position = m_robot_description->findNode("com")->getGlobalPosition(m_joint_positions);
    Eigen::Vector3d com_inertial_position;
    com_inertial_position.noalias() = m_state_posterior.imu_orientation * com_body_position + m_state_posterior.imu_position;
    return com_inertial_position;
}

Eigen::Vector3d SensorFusion::getCOMVelocity() const
{
    Eigen::Vector3d com_body_velocity = m_robot_description->findNode("com")->getGlobalVelocity(m_joint_positions, m_joint_velocities);
    Eigen::Vector3d com_inertial_velocity;
    com_inertial_velocity.noalias() = m_state_posterior.imu_orientation * com_body_velocity + m_state_posterior.imu_velocity;
    return com_inertial_velocity;
}

geometry_msgs::msg::Point SensorFusion::getZMP() const
{
    Eigen::Vector3d zmp_position = m_robot_description->findNode("zmp")->getGlobalPosition(m_joint_positions);
    geometry_msgs::msg::Point zmp;
    zmp.x = zmp_position.x();
    zmp.y = zmp_position.y();
    zmp.z = zmp_position.z();
    return zmp;
}

geometry_msgs::msg::Transform SensorFusion::getRobotTransform() const
{
    geometry_msgs::msg::Transform robot_transform;

    robot_transform.translation.x = m_state_posterior.imu_position.x();
    robot_transform.translation.y = m_state_posterior.imu_position.y();
    robot_transform.translation.z = m_state_posterior.imu_position.z();
    robot_transform.rotation.x = m_state_posterior.imu_orientation.x();
    robot_transform.rotation.y = m_state_posterior.imu_orientation.y();
    robot_transform.rotation.z = m_state_posterior.imu_orientation.z();
    robot_transform.rotation.w = m_state_posterior.imu_orientation.w();

    return robot_transform;
}

std::vector<geometry_msgs::msg::Pose> SensorFusion::getFootPoses() const
{
    std::vector<geometry_msgs::msg::Pose> foot_poses;
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes({ "L_foot", "R_foot" });

    for (long unsigned int i = 0; i < feet_nodes.size(); i++) {
        geometry_msgs::msg::Pose foot_pose;
        Eigen::Vector3d foot_position = feet_nodes[i]->getGlobalPosition(m_joint_positions);
        Eigen::Quaterniond foot_orientation = Eigen::Quaterniond(feet_nodes[i]->getGlobalRotation(m_joint_positions));
        foot_pose.position.x = foot_position.x();
        foot_pose.position.y = foot_position.y();
        foot_pose.position.z = foot_position.z();
        foot_pose.orientation.x = foot_orientation.x();
        foot_pose.orientation.y = foot_orientation.y();
        foot_pose.orientation.z = foot_orientation.z();
        foot_pose.orientation.w = foot_orientation.w();
        foot_poses.push_back(foot_pose);
    }

    return foot_poses;
}

std::vector<double> SensorFusion::getFootContactHeight() const
{
    std::vector<double> foot_contact_height;
    std::vector<RobotNode::SharedPtr> feet_nodes = m_robot_description->findNodes({ "L_foot", "R_foot" });

    for (const auto& foot_node : feet_nodes) {
        Eigen::Vector3d foot_position = foot_node->getGlobalPosition(m_joint_positions);
        foot_position.noalias() 
            = m_state_posterior.imu_orientation * foot_position + m_state_posterior.imu_position;
        foot_contact_height.push_back(foot_position.z());
    }

    return foot_contact_height;
}

Eigen::VectorXd SensorFusion::getPosteriorStateVector() const
{
    Eigen::VectorXd state_vector = getEKFStateVector(m_state_posterior);
    return state_vector;
}

Eigen::Quaterniond SensorFusion::getFilteredOrientation() const
{
    return m_state_posterior.imu_orientation;
}

Eigen::Quaterniond SensorFusion::getExponentialMap(const Eigen::Vector3d& vector) const
{
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d vector_normalized = vector;
    vector_normalized.normalize();

    double angle = vector.norm();
    quaternion.w() = cos(0.5 * angle);
    quaternion.vec() = sin(0.5 * angle) * vector_normalized;
    return quaternion;
}

Eigen::Matrix3d SensorFusion::getSkewSymmetricMatrix(const Eigen::Vector3d& vector) const
{
    Eigen::Matrix3d skew_symmetric_matrix;
    skew_symmetric_matrix <<    0, -vector.z(), vector.y(), 
                                vector.z(), 0, -vector.x(), 
                                -vector.y(), vector.x(), 0;
    return skew_symmetric_matrix;
}

Eigen::MatrixXd SensorFusion::getStateTransitionMatrix(const EKFState& state) const
{
    Eigen::MatrixXd state_transition_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);

    // TODO: Replace these as arguments
    Eigen::Matrix3d ss_expected_measured_angular_velocity = getSkewSymmetricMatrix(calculateExpectedMeasuredAngularVelocity(state.gyroscope_bias));
    Eigen::Matrix3d ss_expected_measured_acceleration = getSkewSymmetricMatrix(calculateExpectedMeasuredAcceleration(state.accelerometer_bias));
    Eigen::Matrix3d rotation_matrix = state.imu_orientation.toRotationMatrix().transpose();

    state_transition_matrix.block<3, 3>(0, 3).noalias() = m_timestep * Eigen::Matrix3d::Identity();
    state_transition_matrix.block<3, 3>(3, 6).noalias() = -rotation_matrix * ss_expected_measured_acceleration * m_timestep;
    state_transition_matrix.block<3, 3>(3, 15).noalias() = -rotation_matrix * m_timestep;
    state_transition_matrix.block<3, 3>(6, 6).noalias() -= ss_expected_measured_angular_velocity * m_timestep;
    state_transition_matrix.block<3, 3>(6, 18).noalias() = -m_timestep * Eigen::Matrix3d::Identity();

    return state_transition_matrix;
}

Eigen::MatrixXd SensorFusion::getObservationModelMatrix(const EKFState& state) const
{
    Eigen::MatrixXd measurement_model_matrix = Eigen::MatrixXd::Zero(MEASUREMENT_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    Eigen::Matrix3d rotation_matrix = state.imu_orientation.toRotationMatrix();

    measurement_model_matrix.block<3, 3>(0, 0).noalias() = -rotation_matrix;
    measurement_model_matrix.block<3, 3>(0, 6).noalias() = getSkewSymmetricMatrix(rotation_matrix * (state.left_foot_position - state.imu_position));
    measurement_model_matrix.block<3, 3>(0, 9).noalias() = rotation_matrix;
    measurement_model_matrix.block<3, 3>(3, 0).noalias() = -rotation_matrix;
    measurement_model_matrix.block<3, 3>(3, 6).noalias() = getSkewSymmetricMatrix(rotation_matrix * (state.right_foot_position - state.imu_position));
    measurement_model_matrix.block<3, 3>(3, 12).noalias() = rotation_matrix;
    measurement_model_matrix.block<3, 3>(6, 6).noalias() = Eigen::Matrix3d::Identity();
    measurement_model_matrix.block<3, 3>(6, 21).noalias() = -(state.imu_orientation * state.left_foot_slippage.inverse()).toRotationMatrix();
    measurement_model_matrix.block<3, 3>(9, 6).noalias() = Eigen::Matrix3d::Identity();
    measurement_model_matrix.block<3, 3>(9, 24).noalias() = -(state.imu_orientation * state.right_foot_slippage.inverse()).toRotationMatrix();

    return measurement_model_matrix;
}

Eigen::MatrixXd SensorFusion::getNoiseJacobianMatrix(const EKFState& state) const
{
    Eigen::MatrixXd noise_jacobian_matrix = Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    Eigen::Matrix3d rotation_matrix = state.imu_orientation.toRotationMatrix().transpose();

    noise_jacobian_matrix.block<3, 3>(0, 0).noalias() = -rotation_matrix;
    noise_jacobian_matrix.block<3, 3>(3, 3).noalias() = -rotation_matrix;
    noise_jacobian_matrix.block<3, 3>(6, 6).noalias() = -Eigen::Matrix3d::Identity();
    noise_jacobian_matrix.block<3, 3>(9, 9).noalias() = rotation_matrix;
    noise_jacobian_matrix.block<3, 3>(12, 12).noalias() = rotation_matrix;

    return noise_jacobian_matrix;
}

void SensorFusion::setTimeStep(const double& timestep)
{
    m_timestep = timestep;
}

void SensorFusion::setProcessNoiseAccelerationVector(const Eigen::Vector3d& process_noise_acceleration)
{
    m_process_noise_acceleration = process_noise_acceleration;
}

void SensorFusion::setProcessNoiseAngularVelocityVector(const Eigen::Vector3d& process_noise_angular_velocity)
{
    m_process_noise_angular_velocity = process_noise_angular_velocity;
}

void SensorFusion::setProcessNoiseCovarianceMatrix(const Eigen::Vector3d& process_noise_velocity,
    const Eigen::Vector3d& process_noise_acceleration, const Eigen::Vector3d& process_noise_angular_velocity,
    const Eigen::Vector3d& process_noise_feet_position, const Eigen::Vector3d& process_noise_accelerometer_bias,
    const Eigen::Vector3d& process_noise_gyroscope_bias, const Eigen::Vector3d& process_noise_feet_slippage)
{
    m_process_noise_covariance_matrix = Eigen::MatrixXd::Zero(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    m_process_noise_covariance_matrix.block<3, 3>(0, 0) = process_noise_velocity.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(3, 3) = process_noise_acceleration.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(6, 6) = process_noise_angular_velocity.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(9, 9) = process_noise_feet_position.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(12, 12) = process_noise_feet_position.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(15, 15) = process_noise_accelerometer_bias.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(18, 18) = process_noise_gyroscope_bias.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(21, 21) = process_noise_feet_slippage.asDiagonal();
    m_process_noise_covariance_matrix.block<3, 3>(24, 24) = process_noise_feet_slippage.asDiagonal();
}

void SensorFusion::setMeasurementNoiseCovarianceMatrix(
    const Eigen::Vector3d& measurement_noise_feet_position, const Eigen::Vector3d& measurement_noise_feet_slippage)
{
    m_measurement_noise_covariance_matrix = Eigen::MatrixXd::Zero(MEASUREMENT_DIMENSION_SIZE, MEASUREMENT_DIMENSION_SIZE);
    m_measurement_noise_covariance_matrix.block<3, 3>(0, 0) = measurement_noise_feet_position.asDiagonal();
    m_measurement_noise_covariance_matrix.block<3, 3>(3, 3) = measurement_noise_feet_position.asDiagonal();
    m_measurement_noise_covariance_matrix.block<3, 3>(6, 6) = measurement_noise_feet_slippage.asDiagonal();
    m_measurement_noise_covariance_matrix.block<3, 3>(9, 9) = measurement_noise_feet_slippage.asDiagonal();
    m_measurement_noise_covariance_matrix.noalias() = m_measurement_noise_covariance_matrix / m_timestep;
}
void SensorFusion::updateProcessNoiseCovarianceMatrix(const EKFState& state_posterior)
{
    Eigen::MatrixXd next_process_noise_covariance_matrix = Eigen::MatrixXd::Zero(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    Eigen::MatrixXd state_transition_matrix, noise_jacobian_matrix;

    state_transition_matrix.noalias() = getStateTransitionMatrix(state_posterior);
    noise_jacobian_matrix.noalias() = getNoiseJacobianMatrix(state_posterior);
    m_process_noise_covariance_matrix.noalias()  = 
        (state_transition_matrix * 
            (noise_jacobian_matrix * 
                (m_process_noise_covariance_matrix * 
                    (noise_jacobian_matrix.transpose() * state_transition_matrix.transpose())
                )
            )
        ) * m_timestep;
}

Eigen::MatrixXd SensorFusion::calculatePriorCovarianceMatrix(const EKFState& state_posterior) const
{
    Eigen::MatrixXd priori_covariance_matrix;
    Eigen::MatrixXd state_transition_matrix = getStateTransitionMatrix(state_posterior);

    priori_covariance_matrix.noalias() = state_transition_matrix * (state_posterior.covariance_matrix * state_transition_matrix.transpose()) + m_process_noise_covariance_matrix;
    return priori_covariance_matrix;
}

Eigen::Vector3d SensorFusion::calculateExpectedMeasuredAcceleration(const Eigen::Vector3d& accelerometer_bias) const
{
    Eigen::Vector3d expected_measured_acceleration;
    expected_measured_acceleration.noalias() = msgToEigenVector3d(m_recent_imu_msg->linear_acceleration) - accelerometer_bias - m_process_noise_acceleration;
    return expected_measured_acceleration;
}

Eigen::Vector3d SensorFusion::calculateExpectedMeasuredAngularVelocity(const Eigen::Vector3d& gyroscope_bias) const
{
    Eigen::Vector3d expected_measured_angular_velocity;
    expected_measured_angular_velocity.noalias() = msgToEigenVector3d(m_recent_imu_msg->angular_velocity) - gyroscope_bias - m_process_noise_angular_velocity;
    return expected_measured_angular_velocity;
}

Eigen::VectorXd SensorFusion::calculateInnovation(const EKFState& state_priori) const
{
    Eigen::VectorXd innovation(MEASUREMENT_DIMENSION_SIZE);
    Eigen::Matrix3d rotation_matrix = state_priori.imu_orientation.toRotationMatrix();

    Eigen::Vector3d residual_left_foot_position, residual_right_foot_position, residual_left_foot_slippage, residual_right_foot_slippage;

    residual_left_foot_position.noalias() = m_robot_description->findNode("L_foot")->getGlobalPosition(m_joint_positions) - rotation_matrix * (state_priori.left_foot_position - state_priori.imu_position);
    residual_right_foot_position.noalias() = m_robot_description->findNode("R_foot")->getGlobalPosition(m_joint_positions) - rotation_matrix * (state_priori.right_foot_position - state_priori.imu_position);
    
    residual_left_foot_slippage.noalias() = rotationMatrixToEulerAngles(m_robot_description->findNode("L_foot")->getGlobalRotation(m_joint_positions)) - quaternionToEulerAngles(state_priori.imu_orientation * state_priori.left_foot_slippage.inverse());
    residual_right_foot_slippage.noalias() = rotationMatrixToEulerAngles(m_robot_description->findNode("R_foot")->getGlobalRotation(m_joint_positions)) - quaternionToEulerAngles(state_priori.imu_orientation * state_priori.right_foot_slippage.inverse());

    // residual_left_foot_position.noalias() = Eigen::Vector3d::Zero();
    // residual_right_foot_position.noalias() = Eigen::Vector3d::Zero();
    
    // residual_left_foot_slippage.noalias() = Eigen::Vector3d::Zero();
    // residual_right_foot_slippage.noalias() = Eigen::Vector3d::Zero();

    innovation <<  residual_left_foot_position, residual_right_foot_position, residual_left_foot_slippage, residual_right_foot_slippage;
    return innovation;
}

Eigen::VectorXd SensorFusion::calculateStateCorrectionVector(const Eigen::MatrixXd& kalman_gain, 
    const Eigen::VectorXd& innovation) const
{
    Eigen::VectorXd state_correction_vector(STATE_DIMENSION_SIZE);
    state_correction_vector.noalias() = kalman_gain * innovation;
    return state_correction_vector;
}

Eigen::MatrixXd SensorFusion::calculateInnovationCovarianceMatrix(const EKFState& state_priori) const
{
    Eigen::MatrixXd innovation_covariance_matrix(MEASUREMENT_DIMENSION_SIZE, MEASUREMENT_DIMENSION_SIZE);
    Eigen::MatrixXd measurement_model_matrix = getObservationModelMatrix(state_priori);

    innovation_covariance_matrix.noalias() = measurement_model_matrix * (state_priori.covariance_matrix * measurement_model_matrix.transpose()) + m_measurement_noise_covariance_matrix;
    return innovation_covariance_matrix;
}

Eigen::MatrixXd SensorFusion::calculateKalmanGain(const EKFState& state_priori, const Eigen::MatrixXd& observation_model_matrix) const
{
    Eigen::MatrixXd kalman_gain;
    Eigen::MatrixXd innovation_covariance_matrix = calculateInnovationCovarianceMatrix(state_priori);

    kalman_gain.noalias() = state_priori.covariance_matrix * (observation_model_matrix.transpose() * innovation_covariance_matrix.inverse());
    return kalman_gain;
}

Eigen::MatrixXd SensorFusion::calculateEstimatedCovarianceMatrix(const EKFState& state_priori, 
    const Eigen::MatrixXd& kalman_gain, const Eigen::MatrixXd& observation_model) const
{
    Eigen::MatrixXd estimated_covariance_matrix = Eigen::MatrixXd(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
    estimated_covariance_matrix.noalias() = (Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE) - kalman_gain * observation_model) * state_priori.covariance_matrix;
    return estimated_covariance_matrix;
}

EKFState SensorFusion::calculatePriorState(const EKFState& state_posterior) const
{
    EKFState state_priori = state_posterior;
    Eigen::Matrix3d rotation_matrix = state_posterior.imu_orientation.toRotationMatrix().transpose();
    Eigen::Vector3d expected_measured_velocity, expected_measured_angular_position;
    expected_measured_velocity.noalias() = m_timestep * (rotation_matrix * calculateExpectedMeasuredAcceleration(state_posterior.accelerometer_bias) + GRAVITY_VECTOR);
    expected_measured_angular_position.noalias() = m_timestep * calculateExpectedMeasuredAngularVelocity(state_posterior.gyroscope_bias);

    state_priori.imu_position.noalias() = state_posterior.imu_position + m_timestep * state_posterior.imu_velocity + 0.5 * m_timestep * expected_measured_velocity;
    state_priori.imu_velocity.noalias() = state_posterior.imu_velocity + expected_measured_velocity;
    state_priori.imu_orientation = getExponentialMap(expected_measured_angular_position) * state_posterior.imu_orientation;
    state_priori.covariance_matrix.noalias() = calculatePriorCovarianceMatrix(state_posterior);
    return state_priori;
}

EKFState SensorFusion::calculatePosteriorState(const EKFState& state_priori) const
{
    EKFState state_posterior;
    Eigen::VectorXd innovation = calculateInnovation(state_priori);
    Eigen::MatrixXd observation_model_matrix = getObservationModelMatrix(state_priori);
    Eigen::MatrixXd kalman_gain = calculateKalmanGain(state_priori, observation_model_matrix);
    Eigen::VectorXd state_correction_vector = calculateStateCorrectionVector(kalman_gain, innovation);
    Eigen::MatrixXd estimated_covariance_matrix = calculateEstimatedCovarianceMatrix(state_priori, kalman_gain, observation_model_matrix);

    state_posterior.imu_position.noalias() = state_priori.imu_position + state_correction_vector.block<3, 1>(0, 0);
    state_posterior.imu_velocity.noalias() = state_priori.imu_velocity + state_correction_vector.block<3, 1>(3, 0);
    state_posterior.imu_orientation = getExponentialMap(state_correction_vector.block<3, 1>(6, 0)) * state_priori.imu_orientation;
    state_posterior.left_foot_position.noalias() = state_priori.left_foot_position + state_correction_vector.block<3, 1>(9, 0);
    state_posterior.right_foot_position.noalias() = state_priori.right_foot_position + state_correction_vector.block<3, 1>(12, 0);
    state_posterior.accelerometer_bias.noalias() = state_priori.accelerometer_bias + state_correction_vector.block<3, 1>(15, 0);
    state_posterior.gyroscope_bias.noalias() = state_priori.gyroscope_bias + state_correction_vector.block<3, 1>(18, 0);
    state_posterior.left_foot_slippage = getExponentialMap(state_correction_vector.block<3, 1>(21, 0)) * state_priori.left_foot_slippage;
    state_posterior.right_foot_slippage = getExponentialMap(state_correction_vector.block<3, 1>(24, 0)) * state_priori.right_foot_slippage;

    return state_posterior;
}

Eigen::VectorXd SensorFusion::getEKFStateVector(const EKFState& state) const
{
    Eigen::VectorXd state_vector(STATE_DIMENSION_SIZE);
    state_vector << state.imu_position, state.imu_velocity, quaternionToEulerAngles(state.imu_orientation),
        state.left_foot_position, state.right_foot_position, state.accelerometer_bias, state.gyroscope_bias,
        quaternionToEulerAngles(state.left_foot_slippage), quaternionToEulerAngles(state.right_foot_slippage);
    return state_vector;
}

bool SensorFusion::hasConverged(const Eigen::Quaterniond& desired_orientation, const double& threshold) const
{
    Eigen::Quaterniond difference = desired_orientation * m_state_posterior.imu_orientation.inverse();
    return atan2(difference.vec().norm(), difference.w()) < threshold;
}

sensor_msgs::msg::Imu::SharedPtr SensorFusion::getFilteredImuMsg() const
{
    sensor_msgs::msg::Imu::SharedPtr filtered_imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    filtered_imu_msg->header.stamp = m_recent_imu_msg->header.stamp;
    filtered_imu_msg->header.frame_id = m_recent_imu_msg->header.frame_id;
    filtered_imu_msg->orientation.x = m_state_posterior.imu_orientation.x();
    filtered_imu_msg->orientation.y = m_state_posterior.imu_orientation.y();
    filtered_imu_msg->orientation.z = m_state_posterior.imu_orientation.z();
    filtered_imu_msg->orientation.w = m_state_posterior.imu_orientation.w();
    filtered_imu_msg->angular_velocity.x = m_state_posterior.imu_velocity.x();
    filtered_imu_msg->angular_velocity.y = m_state_posterior.imu_velocity.y();
    filtered_imu_msg->angular_velocity.z = m_state_posterior.imu_velocity.z();
    filtered_imu_msg->linear_acceleration.x = m_state_posterior.imu_position.x();
    filtered_imu_msg->linear_acceleration.y = m_state_posterior.imu_position.y();
    filtered_imu_msg->linear_acceleration.z = m_state_posterior.imu_position.z();
    return filtered_imu_msg;
}

geometry_msgs::msg::Vector3::SharedPtr SensorFusion::getFilteredOrientationMsg() const
{
    geometry_msgs::msg::Vector3::SharedPtr filtered_orientation = std::make_shared<geometry_msgs::msg::Vector3>();
    Eigen::Vector3d euler_angles = quaternionToEulerAngles(m_state_posterior.imu_orientation);
    filtered_orientation->x = euler_angles.x();
    filtered_orientation->y = euler_angles.y();
    filtered_orientation->z = euler_angles.z();
    return filtered_orientation;
}

Eigen::Vector3d SensorFusion::msgToEigenVector3d(const geometry_msgs::msg::Vector3& vector) const
{
    return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

Eigen::Vector3d SensorFusion::quaternionToEulerAngles(const Eigen::Quaterniond& quaternion) const
{
    return rotationMatrixToEulerAngles(quaternion.toRotationMatrix());    
}

Eigen::Vector3d SensorFusion::rotationMatrixToEulerAngles(const Eigen::Matrix3d& rotation_matrix) const
{
    return rotation_matrix.eulerAngles(EULER_ROLL_AXIS, EULER_PITCH_AXIS, EULER_YAW_AXIS);
    // return rotation_matrix.eulerAngles(EULER_YAW_AXIS, EULER_PITCH_AXIS, EULER_ROLL_AXIS);
}