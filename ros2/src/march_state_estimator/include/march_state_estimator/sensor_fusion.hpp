/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_

// Uncomment to enable debug mode
// #define DEBUG

#include <string>
#include <vector>
#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

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

#define QUATERNION_W 0
#define QUATERNION_X 1
#define QUATERNION_Y 2
#define QUATERNION_Z 3

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define ROTATION_ROLL  0
#define ROTATION_PITCH 1
#define ROTATION_YAW   2
#define CARTESIAN_DIMENSION_SIZE 3
#define SE3_DIMENSION_SIZE 6

#define LEFT_LEG_INDEX 0
#define RIGHT_LEG_INDEX 1
#define NUM_OF_LEGS 2

#define JACOBIAN_POSITION_INDEX 0
#define JACOBIAN_ORIENTATION_INDEX 3

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
    Eigen::VectorXd joint_position;
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
    SensorFusion(double timestep);
    ~SensorFusion() = default;

    inline void estimateState() {
        predictState();
        updateState();
    }

    inline const EKFState& getState() const { return m_state; }

    inline double getPerformanceCost() const { return m_performance_cost; }

    inline void setObservation(const EKFObservation& observation) { m_observation = observation; }

    inline void setTimestep(const double& timestep) { m_timestep = timestep; }

    inline void setJointPosition(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions) {
        // Sort joint positions by joint names order in robot model
        m_joint_position = Eigen::VectorXd::Zero(m_robot_model.nv);
        for (unsigned int i = 0; i < joint_names.size(); i++) {
            m_joint_position[m_robot_model.getJointId(joint_names[i])] = joint_positions[i];
        }
    }

    inline void setProcessNoiseCovarianceMatrix(
        const std::vector<double>& linear_acceleration_covariance, const std::vector<double>& angular_velocity_covariance,
        const std::vector<double>& foot_position_covariance, const std::vector<double>& accelerometer_bias_covariance,
        const std::vector<double>& gyroscope_bias_covariance, const std::vector<double>& foot_slippage_covariance) 
    {
        #ifdef DEBUG
        std::cout << "Linear acceleration covariance: " << Eigen::Map<const Eigen::VectorXd>(linear_acceleration_covariance.data(), linear_acceleration_covariance.size()).transpose() << std::endl;
        std::cout << "Angular velocity covariance: " << Eigen::Map<const Eigen::VectorXd>(angular_velocity_covariance.data(), angular_velocity_covariance.size()).transpose() << std::endl;
        std::cout << "Foot position covariance: " << Eigen::Map<const Eigen::VectorXd>(foot_position_covariance.data(), foot_position_covariance.size()).transpose() << std::endl;
        std::cout << "Accelerometer bias covariance: " << Eigen::Map<const Eigen::VectorXd>(accelerometer_bias_covariance.data(), accelerometer_bias_covariance.size()).transpose() << std::endl;
        std::cout << "Gyroscope bias covariance: " << Eigen::Map<const Eigen::VectorXd>(gyroscope_bias_covariance.data(), gyroscope_bias_covariance.size()).transpose() << std::endl;
        std::cout << "Foot slippage covariance: " << Eigen::Map<const Eigen::VectorXd>(foot_slippage_covariance.data(), foot_slippage_covariance.size()).transpose() << std::endl;
        #endif

        m_process_noise_covariance_matrix = Eigen::MatrixXd::Zero(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE);
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_VELOCITY, STATE_INDEX_VELOCITY)
            = m_timestep *Eigen::Map<const Eigen::VectorXd>(linear_acceleration_covariance.data(), linear_acceleration_covariance.size()).asDiagonal();
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_POSITION, STATE_INDEX_POSITION)
            = 0.5 * m_timestep * m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_VELOCITY, STATE_INDEX_VELOCITY);
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_ORIENTATION, STATE_INDEX_ORIENTATION)
            = m_timestep * Eigen::Map<const Eigen::VectorXd>(angular_velocity_covariance.data(), angular_velocity_covariance.size()).asDiagonal();
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_LEFT_FOOT_POSITION, STATE_INDEX_LEFT_FOOT_POSITION)
            = Eigen::Map<const Eigen::VectorXd>(foot_position_covariance.data(), foot_position_covariance.size()).asDiagonal();
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_RIGHT_FOOT_POSITION, STATE_INDEX_RIGHT_FOOT_POSITION)
            = Eigen::Map<const Eigen::VectorXd>(foot_position_covariance.data(), foot_position_covariance.size()).asDiagonal();
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_ACCELEROMETER_BIAS, STATE_INDEX_ACCELEROMETER_BIAS)
            = Eigen::Map<const Eigen::VectorXd>(accelerometer_bias_covariance.data(), accelerometer_bias_covariance.size()).asDiagonal();
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_GYROSCOPE_BIAS, STATE_INDEX_GYROSCOPE_BIAS)
            = Eigen::Map<const Eigen::VectorXd>(gyroscope_bias_covariance.data(), gyroscope_bias_covariance.size()).asDiagonal();
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_LEFT_SLIPPAGE, STATE_INDEX_LEFT_SLIPPAGE)
            = Eigen::Map<const Eigen::VectorXd>(foot_slippage_covariance.data(), foot_slippage_covariance.size()).asDiagonal(); 
        m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_RIGHT_SLIPPAGE, STATE_INDEX_RIGHT_SLIPPAGE)
            = Eigen::Map<const Eigen::VectorXd>(foot_slippage_covariance.data(), foot_slippage_covariance.size()).asDiagonal();
        
        m_process_noise_foot_position = Eigen::Map<const Eigen::VectorXd>(foot_position_covariance.data(), foot_position_covariance.size()).asDiagonal();
        m_process_noise_foot_slippage = Eigen::Map<const Eigen::VectorXd>(foot_slippage_covariance.data(), foot_slippage_covariance.size()).asDiagonal();

        #ifdef DEBUG
        std::cout << "Process noise covariance matrix:\n" << m_process_noise_covariance_matrix << std::endl;
        #endif
    }

    inline void setObservationNoiseCovarianceMatrix(const std::vector<double>& position_covariance,
        const std::vector<double>& slippage_covariance, const std::vector<double>& joint_covariance)
    {
        #ifdef DEBUG
        std::cout << "Position covariance: " << Eigen::Map<const Eigen::VectorXd>(position_covariance.data(), position_covariance.size()).transpose() << std::endl;
        std::cout << "Slippage covariance: " << Eigen::Map<const Eigen::VectorXd>(slippage_covariance.data(), slippage_covariance.size()).transpose() << std::endl;
        std::cout << "Joint covariance: " << Eigen::Map<const Eigen::VectorXd>(joint_covariance.data(), joint_covariance.size()).transpose() << std::endl;
        #endif

        m_observation_noise_covariance_matrix = Eigen::MatrixXd::Zero(MEASUREMENT_DIMENSION_SIZE, MEASUREMENT_DIMENSION_SIZE);
        m_observation_noise_covariance_position_matrix = Eigen::Map<const Eigen::VectorXd>(position_covariance.data(), position_covariance.size()).asDiagonal();
        m_observation_noise_covariance_slippage_matrix = Eigen::Map<const Eigen::VectorXd>(slippage_covariance.data(), slippage_covariance.size()).asDiagonal();
        m_observation_noise_covariance_joint_matrix = Eigen::Map<const Eigen::VectorXd>(joint_covariance.data(), joint_covariance.size()).asDiagonal();
        
        m_observation_noise_covariance_left_position_matrix = m_observation_noise_covariance_position_matrix;
        m_observation_noise_covariance_right_position_matrix = m_observation_noise_covariance_position_matrix;
        m_observation_noise_covariance_right_slippage_matrix = m_observation_noise_covariance_slippage_matrix;
        m_observation_noise_covariance_left_slippage_matrix = m_observation_noise_covariance_slippage_matrix;

        m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_POSITION, MEASUREMENT_INDEX_LEFT_POSITION)
            = m_observation_noise_covariance_left_position_matrix;
        m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_POSITION, MEASUREMENT_INDEX_RIGHT_POSITION)
            = m_observation_noise_covariance_right_position_matrix;
        m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_LEFT_SLIPPAGE, MEASUREMENT_INDEX_LEFT_SLIPPAGE)
            = m_observation_noise_covariance_left_slippage_matrix;
        m_observation_noise_covariance_matrix.block<3, 3>(MEASUREMENT_INDEX_RIGHT_SLIPPAGE, MEASUREMENT_INDEX_RIGHT_SLIPPAGE)
            = m_observation_noise_covariance_right_slippage_matrix;

        #ifdef DEBUG
        std::cout << "Observation noise covariance matrix:\n" << m_observation_noise_covariance_matrix << std::endl;
        #endif
    }

    inline void updateStanceLeg(uint8_t current_stance_leg) {
        // m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_LEFT_FOOT_POSITION, STATE_INDEX_LEFT_FOOT_POSITION)
        //     = (current_stance_leg & 0b01) * m_process_noise_foot_position + (~current_stance_leg & 0b01) * computeVeryLargeMatrix3d();
        // m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_RIGHT_FOOT_POSITION, STATE_INDEX_RIGHT_FOOT_POSITION)
        //     = ((current_stance_leg >> 1) & 0b01) * m_process_noise_foot_position + ((~current_stance_leg >> 1) & 0b01) *computeVeryLargeMatrix3d();
        // m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_LEFT_SLIPPAGE, STATE_INDEX_LEFT_SLIPPAGE)
        //     = (current_stance_leg & 0b01) * m_process_noise_foot_slippage + (~current_stance_leg & 0b01) * computeVeryLargeMatrix3d();
        // m_process_noise_covariance_matrix.block<3, 3>(STATE_INDEX_RIGHT_SLIPPAGE, STATE_INDEX_RIGHT_SLIPPAGE)
        //     = ((current_stance_leg >> 1) & 0b01) * m_process_noise_foot_slippage + ((~current_stance_leg >> 1) & 0b01) * computeVeryLargeMatrix3d();

        #ifdef DEBUG
        std::cout << "Current stance leg: " << (int)current_stance_leg << std::endl;
        std::cout << "Process noise covariance matrix:\n" << m_process_noise_covariance_matrix << std::endl;
        #endif

        m_observation_noise_covariance_left_position_matrix
            = (current_stance_leg & 0b01) * m_observation_noise_covariance_position_matrix + (~current_stance_leg & 0b01) * computeVeryLargeMatrix3d();
        m_observation_noise_covariance_right_position_matrix
            = ((current_stance_leg & 0b10) >> 1) * m_observation_noise_covariance_position_matrix + (~(current_stance_leg & 0b10) >> 1) * computeVeryLargeMatrix3d();
        m_observation_noise_covariance_left_slippage_matrix
            = (current_stance_leg & 0b01) * m_observation_noise_covariance_slippage_matrix + (~current_stance_leg & 0b01) * computeVeryLargeMatrix3d();
        m_observation_noise_covariance_right_slippage_matrix
            = ((current_stance_leg & 0b10) >> 1) * m_observation_noise_covariance_slippage_matrix + (~(current_stance_leg & 0b10) >> 1) * computeVeryLargeMatrix3d();

        #ifdef DEBUG
        std::cout << "Observation noise covariance matrix:\n" << m_observation_noise_covariance_matrix << std::endl;
        #endif
    }

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
        Eigen::MatrixXd prior_covariance_matrix;
        prior_covariance_matrix.noalias() = m_dynamics_matrix * m_state.covariance_matrix * m_dynamics_matrix.transpose() + m_process_noise_covariance_matrix;
        try {
            Eigen::LLT<Eigen::MatrixXd> llt(prior_covariance_matrix);
            if (llt.info() == Eigen::NumericalIssue) {
                throw std::runtime_error("Prior covariance matrix is not positive semi-definite.");
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
        #ifdef DEBUG
        std::cout << "Prior covariance matrix:\n" << prior_covariance_matrix << std::endl;
        #endif
        return prior_covariance_matrix;
    }

    inline const Eigen::MatrixXd computePosteriorCovarianceMatrix() const {
        Eigen::MatrixXd posterior_covariance_matrix;
        posterior_covariance_matrix.noalias() = (Eigen::MatrixXd::Identity(STATE_DIMENSION_SIZE, STATE_DIMENSION_SIZE) - m_kalman_gain * m_observation_matrix) * m_state.covariance_matrix;
        try {
            Eigen::LLT<Eigen::MatrixXd> llt(posterior_covariance_matrix);
            if (llt.info() == Eigen::NumericalIssue) {
                throw std::runtime_error("Posterior covariance matrix is not positive semi-definite.");
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
        #ifdef DEBUG
        std::cout << "Posterior covariance matrix:\n" << posterior_covariance_matrix << std::endl;
        #endif
        return posterior_covariance_matrix;
    }

    inline void computeInnovationCovarianceMatrix() {
        m_innovation_covariance_matrix.noalias() = m_observation_matrix * m_state.covariance_matrix * m_observation_matrix.transpose() + m_observation_noise_covariance_matrix;
        try {
            Eigen::LLT<Eigen::MatrixXd> llt(m_innovation_covariance_matrix);
            if (llt.info() == Eigen::NumericalIssue) {
                throw std::runtime_error("Innovation covariance matrix is not positive semi-definite.");
            }
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
        #ifdef DEBUG
        std::cout << "Innovation covariance matrix:\n" << m_innovation_covariance_matrix << std::endl;
        #endif
    }

    inline void computeKalmanGain() {
        m_kalman_gain.noalias() = m_state.covariance_matrix * m_observation_matrix.transpose() * m_innovation_covariance_matrix.inverse();
        #ifdef DEBUG
        std::cout << "Kalman gain matrix:\n" << m_kalman_gain << std::endl;
        #endif
    }

    inline void computeProcessNoiseCovarianceMatrix() {
        // Eigen::MatrixXd noise_jacobian_matrix = computeNoiseJacobianMatrix();
        // m_process_noise_covariance_matrix.noalias() 
        //     = m_dynamics_matrix * noise_jacobian_matrix * m_process_noise_covariance_matrix 
        //         * noise_jacobian_matrix.transpose() * m_dynamics_matrix.transpose() * m_timestep;
        #ifdef DEBUG
        std::cout << "Process noise covariance matrix:\n" << m_process_noise_covariance_matrix << std::endl;
        #endif
    }

    inline void computePerformanceCost(const Eigen::VectorXd& innovation) {
        m_performance_cost = innovation.transpose() * m_innovation_covariance_matrix.inverse() * innovation;
        #ifdef DEBUG
        std::cout << "Performance cost:\n" << m_performance_cost << std::endl;
        #endif
    }

    const Eigen::VectorXd computeInnovation() const;
    const Eigen::MatrixXd computeNoiseJacobianMatrix() const;
    void computeDynamicsMatrix();
    void computeObservationMatrix();
    void computeObservationNoiseCovarianceMatrix();

    inline const Eigen::Matrix3d computeVeryLargeMatrix3d() const { return Eigen::Matrix3d::Identity() * 1e23; }
    inline const Eigen::Vector3d computeEulerAngles(const Eigen::Quaterniond& orientation) const {
        Eigen::Quaterniond q = orientation;
        q.normalize();
        return q.toRotationMatrix().eulerAngles(ROTATION_ROLL, ROTATION_PITCH, ROTATION_YAW);
    }
    const Eigen::Quaterniond computeExponentialMap(const Eigen::Vector3d& vector) const;
    const Eigen::Matrix3d computeSkewSymmetricMatrix(const Eigen::Vector3d& vector) const;

    double m_timestep;
    double m_performance_cost;

    EKFState m_state;
    EKFObservation m_observation;
    Eigen::MatrixXd m_dynamics_matrix;
    Eigen::MatrixXd m_observation_matrix;
    Eigen::MatrixXd m_innovation_covariance_matrix;
    Eigen::MatrixXd m_kalman_gain;
    Eigen::MatrixXd m_process_noise_covariance_matrix;
    Eigen::Matrix3d m_process_noise_foot_position;
    Eigen::Matrix3d m_process_noise_foot_slippage;
    Eigen::MatrixXd m_observation_noise_covariance_matrix;
    Eigen::Matrix3d m_observation_noise_covariance_left_position_matrix;
    Eigen::Matrix3d m_observation_noise_covariance_right_position_matrix;
    Eigen::Matrix3d m_observation_noise_covariance_left_slippage_matrix;
    Eigen::Matrix3d m_observation_noise_covariance_right_slippage_matrix;
    Eigen::MatrixXd m_observation_noise_covariance_joint_matrix;

    Eigen::Matrix3d m_observation_noise_covariance_position_matrix;
    Eigen::Matrix3d m_observation_noise_covariance_slippage_matrix;

    pinocchio::Model m_robot_model;
    std::unique_ptr<pinocchio::Data> m_robot_data;

    Eigen::VectorXd m_joint_position;
    int m_joint_idx[NUM_OF_LEGS];

    friend class SensorFusionTest;
};

#endif  // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_HPP_