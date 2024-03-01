/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <unordered_map>
#include <utility>
#include <vector>
#include <chrono>
#include <random>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "march_state_estimator/sensor_fusion.hpp"

class SensorFusionTest : public ::testing::Test {
public:
    SensorFusionTest() = default;
    ~SensorFusionTest() override = default;

protected:
    void TearDown() override
    {
        m_sensor_fusion.reset();
        m_robot_description.reset();
    }

    void setupRotationalTestSetup()
    {
        std::string yaml_filename = "robot_definition-rotational_test_setup.yaml";
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_sensor_fusion = std::make_unique<SensorFusion>(m_robot_description);

        std::vector<std::string> joint_names = { "bar" };
        m_sensor_fusion->configureJointNames(joint_names);
    }

    void setupHennieWithKoen()
    {
        std::string yaml_filename = "robot_definition-hennie_with_koen.yaml";
        m_robot_description = std::make_shared<RobotDescription>(yaml_filename);
        m_sensor_fusion = std::make_unique<SensorFusion>(m_robot_description);
        m_sensor_fusion->configureJointNames(m_joint_names);
    }

    RobotNode::JointNameToValueMap createJointValueForRotationalTestSetup(const double& joint_value)
    {
        RobotNode::JointNameToValueMap joint_zero_values = { { "bar", joint_value } };
        return joint_zero_values;
    }

    RobotNode::JointNameToValueMap createZeroJointValueForRotationalTestSetup()
    {
        return createJointValueForRotationalTestSetup(0.0);
    }

    RobotNode::JointNameToValueMap createJointValueForHenieWithKoen(const std::vector<double>& joint_values)
    {
        RobotNode::JointNameToValueMap joint_map;
        for (unsigned int i = 0; i < m_joint_names.size(); i++) {
            joint_map[m_joint_names[i]] = joint_values[i];
        }
        return joint_map;
    }

    geometry_msgs::msg::Quaternion createLeveledQuaternion()
    {
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.w = 1.0;
        quaternion.x = 0.0;
        quaternion.y = 0.0;
        quaternion.z = 0.0;
        return quaternion;
    }

    sensor_msgs::msg::JointState::SharedPtr createEmptyJointStateForHennieWithKoen()
    {
        std::vector<double> zero_values(m_joint_names.size(), 0.0);
        sensor_msgs::msg::JointState::SharedPtr joint_state = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state->name = m_joint_names;
        joint_state->position = zero_values;
        joint_state->velocity = zero_values;
        joint_state->effort = zero_values;
        return joint_state;
    }

    geometry_msgs::msg::Point createZeroPoint()
    {
        geometry_msgs::msg::Point point;
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
        return point;
    }

    void displayActualAndExpectedVectors(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected)
    {
        std::cout << "Actual: " << std::endl << actual.transpose() << std::endl;
        std::cout << "Expected: " << std::endl << expected.transpose() << std::endl;
        std::cout << "Difference: " << std::endl << (actual - expected).transpose() << std::endl;
    }

    void displayActualAndExpectedMatrices(const Eigen::MatrixXd& actual, const Eigen::MatrixXd& expected)
    {
        std::cout << "Actual: " << std::endl << actual << std::endl;
        std::cout << "Expected: " << std::endl << expected << std::endl;
        std::cout << "Difference: " << std::endl << (actual - expected) << std::endl;
    }

    void setupZeroSensorFusionNoise()
    {
        m_sensor_fusion->setTimeStep(m_timestep);
        m_sensor_fusion->setProcessNoiseAccelerationVector(Eigen::Vector3d::Zero());
        m_sensor_fusion->setProcessNoiseAngularVelocityVector(Eigen::Vector3d::Zero());
        m_sensor_fusion->setProcessNoiseCovarianceMatrix(Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        m_sensor_fusion->setMeasurementNoiseCovarianceMatrix(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }

    void setupDummySensorFusionNoise()
    {
        m_sensor_fusion->setTimeStep(m_timestep);
        m_sensor_fusion->setProcessNoiseAccelerationVector(Eigen::Vector3d(0.01, 0.01, 0.01));
        m_sensor_fusion->setProcessNoiseAngularVelocityVector(Eigen::Vector3d(0.01, 0.01, 0.01));
        m_sensor_fusion->setProcessNoiseCovarianceMatrix(Eigen::Vector3d(0.01, 0.01, 0.01),
            Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.01, 0.01, 0.01),
            Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.01, 0.01, 0.01));
        m_sensor_fusion->setMeasurementNoiseCovarianceMatrix(Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.01, 0.01, 0.01));
    }

    void testNoiselessKalmanFilterConvergenceForHennieWithKoen(const double& epsilon, const double& duration)
    {
        setupHennieWithKoen();
        const std::chrono::duration<double> time_limit = std::chrono::duration<double>(duration);
        const Eigen::Quaterniond desired_orientation = Eigen::Quaterniond(1, 0, 0, 0);
        const double imu_update_rate = 100.0; // Hz

        sensor_msgs::msg::Imu::SharedPtr imu = std::make_shared<sensor_msgs::msg::Imu>();
        imu->linear_acceleration.x = 0.0;
        imu->linear_acceleration.y = 0.0;
        imu->linear_acceleration.z = 0.0;
        imu->angular_velocity.x = 0.0;
        imu->angular_velocity.y = 0.0;
        imu->angular_velocity.z = 0.0;
        imu->orientation.x = 0.0;
        imu->orientation.y = 0.707106781;
        imu->orientation.z = 0.0;
        imu->orientation.w = 0.707106781;
        m_sensor_fusion->updateImu(imu);

        sensor_msgs::msg::JointState::SharedPtr joint_state = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state->name = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};
        joint_state->position = {0, 0, 0, 0, 0, 0, 0, 0};
        joint_state->velocity = {0, 0, 0, 0, 0, 0, 0, 0};
        joint_state->effort = {0, 0, 0, 0, 0, 0, 0, 0};
        m_sensor_fusion->updateJointState(joint_state);

        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        do {
            if (std::chrono::steady_clock::now() - end_time > std::chrono::duration<double>(1.0 / imu_update_rate)) {
                m_sensor_fusion->updateKalmanFilter();
                end_time = std::chrono::steady_clock::now();
            }
        } while (std::chrono::steady_clock::now() - start_time < time_limit && !m_sensor_fusion->hasConverged(desired_orientation, epsilon));

        std::cout << "Converged in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() << "ms" << std::endl;
        std::cout << "Actual orientation: " << m_sensor_fusion->getFilteredOrientation().w() << ", " << m_sensor_fusion->getFilteredOrientation().x() << ", " 
                    << m_sensor_fusion->getFilteredOrientation().y() << ", " << m_sensor_fusion->getFilteredOrientation().z() << std::endl;
        std::cout << "Desired orientation: " << desired_orientation.w() << ", " << desired_orientation.x() << ", " << desired_orientation.y() << ", " << desired_orientation.z() << std::endl;
        ASSERT_TRUE(m_sensor_fusion->hasConverged(desired_orientation, epsilon));
        ASSERT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count(), time_limit.count());
    }

    void testNoisyKalmanFilterConvergenceForHennieWithKoen(const double& epsilon, const double& duration, const unsigned int iterations)
    {
        setupHennieWithKoen();
        const std::chrono::duration<double> time_limit = std::chrono::duration<double>(duration);
        const Eigen::Quaterniond desired_orientation = Eigen::Quaterniond(1, 0, 0, 0);
        const double imu_update_rate = 100.0; // Hz
        const double gyroscope_noise = 0.118174;
        const double accelerometer_noise = 0.0000216;

        // TODO: Move into member variable
        std::default_random_engine m_random_engine;
        std::normal_distribution<double> gyroscope_noise_distribution(0.0, gyroscope_noise);
        std::normal_distribution<double> accelerometer_noise_distribution(0.0, accelerometer_noise);

        sensor_msgs::msg::Imu::SharedPtr imu = std::make_shared<sensor_msgs::msg::Imu>();
        imu->linear_acceleration.x = 0.0;
        imu->linear_acceleration.y = 0.0;
        imu->linear_acceleration.z = 0.0;
        imu->angular_velocity.x = 0.0;
        imu->angular_velocity.y = 0.0;
        imu->angular_velocity.z = 0.0;
        imu->orientation.x = 0.0;
        imu->orientation.y = 0.707106781;
        imu->orientation.z = 0.0;
        imu->orientation.w = 0.707106781;
        m_sensor_fusion->updateImu(imu);

        sensor_msgs::msg::JointState::SharedPtr joint_state = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state->name = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};
        joint_state->position = {0, 0, 0, 0, 0, 0, 0, 0};
        joint_state->velocity = {0, 0, 0, 0, 0, 0, 0, 0};
        joint_state->effort = {0, 0, 0, 0, 0, 0, 0, 0};
        m_sensor_fusion->updateJointState(joint_state);


        unsigned int successful_iterations = 0;
        for (unsigned int i = 0; i < iterations; i++)
        {
            std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
            do {
                if (std::chrono::steady_clock::now() - end_time > std::chrono::duration<double>(1.0 / imu_update_rate)) {

                    sensor_msgs::msg::Imu::SharedPtr noisy_imu = std::make_shared<sensor_msgs::msg::Imu>();
                    noisy_imu->angular_velocity.x += gyroscope_noise_distribution(m_random_engine);
                    noisy_imu->angular_velocity.y += gyroscope_noise_distribution(m_random_engine);
                    noisy_imu->angular_velocity.z += gyroscope_noise_distribution(m_random_engine);
                    noisy_imu->linear_acceleration.x += accelerometer_noise_distribution(m_random_engine);
                    noisy_imu->linear_acceleration.y += accelerometer_noise_distribution(m_random_engine);
                    noisy_imu->linear_acceleration.z += accelerometer_noise_distribution(m_random_engine);
                    m_sensor_fusion->updateImu(noisy_imu);

                    m_sensor_fusion->updateKalmanFilter();
                    end_time = std::chrono::steady_clock::now();

                    if (m_sensor_fusion->hasConverged(desired_orientation, epsilon)) {
                        successful_iterations++;
                        break;
                    }
                }
            } while (std::chrono::steady_clock::now() - start_time < time_limit);

            std::cout << "Iteration: " << i << std::endl;
            std::cout << "Converged in " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() << "ms" << std::endl;
            std::cout << "Actual orientation: " << m_sensor_fusion->getFilteredOrientation().w() << ", " << m_sensor_fusion->getFilteredOrientation().x() << ", " 
                        << m_sensor_fusion->getFilteredOrientation().y() << ", " << m_sensor_fusion->getFilteredOrientation().z() << std::endl;
            std::cout << "Desired orientation: " << desired_orientation.w() << ", " << desired_orientation.x() << ", " << desired_orientation.y() << ", " << desired_orientation.z() << std::endl;
        }
        ASSERT_GE(successful_iterations, 0.8 * iterations);
    }

    std::shared_ptr<RobotDescription> m_robot_description;
    std::unique_ptr<SensorFusion> m_sensor_fusion;

    // Hennie with Koen
    std::vector<std::string> m_joint_names = {
        "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle",
        "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"
    };
    double m_timestep = 0.01;

};

TEST_F(SensorFusionTest, test_should_create_sensor_fusion_instance_and_setup_hennie_with_koen)
{
    ASSERT_NO_FATAL_FAILURE(setupHennieWithKoen());
}

TEST_F(SensorFusionTest, test_should_update_imu_for_hennie_with_koen)
{
    setupHennieWithKoen();
    sensor_msgs::msg::Imu::SharedPtr imu = std::make_shared<sensor_msgs::msg::Imu>();
    ASSERT_NO_FATAL_FAILURE(m_sensor_fusion->updateImu(imu));
}

TEST_F(SensorFusionTest, test_should_update_joint_state_for_hennie_with_koen)
{
    setupHennieWithKoen();
    sensor_msgs::msg::JointState::SharedPtr joint_state = createEmptyJointStateForHennieWithKoen();
    ASSERT_NO_FATAL_FAILURE(m_sensor_fusion->updateJointState(joint_state));
}

/************************************************
 *
 * Extended Kalman Filter Tests
 * 
 ************************************************/

TEST_F(SensorFusionTest, test_should_update_stance_leg_for_hennie_with_koen)
{
    setupHennieWithKoen();
    geometry_msgs::msg::Point left_foot_position = createZeroPoint();
    geometry_msgs::msg::Point right_foot_position = createZeroPoint();
    ASSERT_NO_FATAL_FAILURE(m_sensor_fusion->updateStanceLeg(&left_foot_position, &right_foot_position));
}

TEST_F(SensorFusionTest, test_should_calculate_exponential_map_and_get_correct_results_for_hennie_with_koen)
{
    setupHennieWithKoen();
    Eigen::Vector3d vector = { M_PI_4, M_PI_2, M_PI };
    Eigen::Quaterniond expected_exponential_map = Eigen::Quaterniond(
        -0.226786501315975431830791787866308922715182137931097988559879999,
        0.2125321132957221365648329909063146676566730739751842464144819159,
        0.4250642265914442731296659818126293353133461479503684928289638319, 
        0.8501284531828885462593319636252586706266922959007369856579276639);
    Eigen::Quaterniond actual_exponential_map = m_sensor_fusion->getExponentialMap(vector);
    ASSERT_TRUE(expected_exponential_map.isApprox(actual_exponential_map, 1e-13));
}

TEST_F(SensorFusionTest, test_should_calculate_skew_symmetric_matrix_and_get_correct_results_for_hennie_with_koen)
{
    setupHennieWithKoen();
    Eigen::Vector3d vector = { 1.0, 2.0, 3.0 };
    Eigen::Matrix3d expected_skew_symmetric_matrix;
    expected_skew_symmetric_matrix << 0.0, -3.0, 2.0,
                                      3.0, 0.0, -1.0,
                                      -2.0, 1.0, 0.0;
    Eigen::Matrix3d actual_skew_symmetric_matrix = m_sensor_fusion->getSkewSymmetricMatrix(vector);
    ASSERT_TRUE(expected_skew_symmetric_matrix.isApprox(actual_skew_symmetric_matrix, 1e-13));
}

TEST_F(SensorFusionTest, test_should_calculate_expected_measured_acceleration_for_hennie_with_koen)
{
    setupHennieWithKoen();
    setupZeroSensorFusionNoise();
    Eigen::Vector3d accelerometer_bias = Eigen::Vector3d::Zero();
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());

    Eigen::Vector3d expected_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual_acceleration = m_sensor_fusion->calculateExpectedMeasuredAcceleration(accelerometer_bias);
    ASSERT_TRUE(expected_acceleration.isApprox(actual_acceleration, 1e-13));
}

TEST_F(SensorFusionTest, test_should_calculate_expected_measured_angular_velocity_for_hennie_with_koen)
{
    setupHennieWithKoen();
    setupZeroSensorFusionNoise();
    Eigen::Vector3d gyroscope_bias = Eigen::Vector3d::Zero();
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());

    Eigen::Vector3d expected_angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual_angular_velocity = m_sensor_fusion->calculateExpectedMeasuredAngularVelocity(gyroscope_bias);
    ASSERT_TRUE(expected_angular_velocity.isApprox(actual_angular_velocity, 1e-13));
}

TEST_F(SensorFusionTest, test_should_calculate_zero_prior_state_for_hennie_with_koen)
{
    setupHennieWithKoen();
    setupZeroSensorFusionNoise();
    EKFState state_posterior;
    m_sensor_fusion->setProcessNoiseAccelerationVector(Eigen::Vector3d::Zero());
    m_sensor_fusion->setProcessNoiseAngularVelocityVector(Eigen::Vector3d::Zero());
    m_sensor_fusion->setProcessNoiseCovarianceMatrix(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
        Eigen::Vector3d::Zero());
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());

    EKFState expected_prior_state = state_posterior;
    expected_prior_state.imu_position = Eigen::Vector3d(0.0, 0.0, -0.0004903325);
    expected_prior_state.imu_velocity = Eigen::Vector3d(0.0, 0.0, -0.0980665);

    EKFState actual_prior_state = m_sensor_fusion->calculatePriorState(state_posterior);    
    displayActualAndExpectedVectors(actual_prior_state.imu_position, expected_prior_state.imu_position);
    displayActualAndExpectedVectors(actual_prior_state.imu_velocity, expected_prior_state.imu_velocity);
    displayActualAndExpectedVectors(actual_prior_state.left_foot_position, expected_prior_state.left_foot_position);
    displayActualAndExpectedVectors(actual_prior_state.right_foot_position, expected_prior_state.right_foot_position);
    displayActualAndExpectedVectors(actual_prior_state.accelerometer_bias, expected_prior_state.accelerometer_bias);
    displayActualAndExpectedVectors(actual_prior_state.gyroscope_bias, expected_prior_state.gyroscope_bias);

    ASSERT_TRUE(expected_prior_state.imu_position.isApprox(actual_prior_state.imu_position, 1e-13));
    ASSERT_TRUE(expected_prior_state.imu_velocity.isApprox(actual_prior_state.imu_velocity, 1e-13));
    ASSERT_TRUE(expected_prior_state.imu_orientation.isApprox(actual_prior_state.imu_orientation, 1e-13));
    ASSERT_TRUE(expected_prior_state.left_foot_position.isApprox(actual_prior_state.left_foot_position, 1e-13));
    ASSERT_TRUE(expected_prior_state.right_foot_position.isApprox(actual_prior_state.right_foot_position, 1e-13));
    ASSERT_TRUE(expected_prior_state.accelerometer_bias.isApprox(actual_prior_state.accelerometer_bias, 1e-13));
    ASSERT_TRUE(expected_prior_state.gyroscope_bias.isApprox(actual_prior_state.gyroscope_bias, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_vectorize_EKF_state_struct)
{
    setupHennieWithKoen();
    EKFState state;

    Eigen::VectorXd expected_vectorized_state = Eigen::VectorXd::Zero(27);
    Eigen::VectorXd actual_vectorized_state = m_sensor_fusion->getEKFStateVector(state);

    displayActualAndExpectedVectors(actual_vectorized_state, expected_vectorized_state);
    ASSERT_EQ(expected_vectorized_state.size(), actual_vectorized_state.size());
    ASSERT_TRUE(expected_vectorized_state.isApprox(actual_vectorized_state, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_get_state_transition_matrix)
{
    setupHennieWithKoen();
    setupDummySensorFusionNoise();
    EKFState state;

    sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->angular_velocity.x = 4.0;
    imu_msg->angular_velocity.y = 5.0;
    imu_msg->angular_velocity.z = 6.0;
    imu_msg->linear_acceleration.x = 1.0;
    imu_msg->linear_acceleration.y = 2.0;
    imu_msg->linear_acceleration.z = 3.0;
    m_sensor_fusion->updateImu(imu_msg);

    Eigen::MatrixXd expected_state_transition_matrix = Eigen::MatrixXd::Zero(27, 27);
    expected_state_transition_matrix.row(0) << 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(1) << 0.0, 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(2) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(3) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0299, -0.0199, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(4) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0299, 0.0, 0.0099, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(5) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0199, -0.0099, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0599, -0.0499, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0599, 1.0, 0.0399, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(8) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0499, -0.0399, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(9) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(10) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(11) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(12) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(13) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(14) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(15) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(16) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(17) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(18) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(19) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(20) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(21) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(22) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    expected_state_transition_matrix.row(23) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    expected_state_transition_matrix.row(24) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    expected_state_transition_matrix.row(25) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    expected_state_transition_matrix.row(26) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Eigen::MatrixXd actual_state_transition_matrix = m_sensor_fusion->getStateTransitionMatrix(state);

    displayActualAndExpectedMatrices(actual_state_transition_matrix, expected_state_transition_matrix);
    ASSERT_EQ(expected_state_transition_matrix.rows(), actual_state_transition_matrix.rows());
    ASSERT_EQ(expected_state_transition_matrix.cols(), actual_state_transition_matrix.cols());
    ASSERT_TRUE(expected_state_transition_matrix.isApprox(actual_state_transition_matrix, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_calculate_identity_prior_covariance_matrix_with_process_noise)
{
    setupHennieWithKoen();
    setupDummySensorFusionNoise();
    EKFState state;
    sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->angular_velocity.x = 4.0;
    imu_msg->angular_velocity.y = 5.0;
    imu_msg->angular_velocity.z = 6.0;
    imu_msg->linear_acceleration.x = 1.0;
    imu_msg->linear_acceleration.y = 2.0;
    imu_msg->linear_acceleration.z = 3.0;
    m_sensor_fusion->updateImu(imu_msg);

    Eigen::MatrixXd expected_prior_covariance_matrix = Eigen::MatrixXd::Identity(27, 27);
    expected_prior_covariance_matrix.row(0) << 1.0101, 0, 0, 0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(1) << 0, 1.0101, 0, 0, 0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(2) << 0, 0, 1.0101, 0, 0, 0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    expected_prior_covariance_matrix.row(3) << 0.0100, 0, 0, 1.011390020000000, -0.000197010000000, -0.000296010000000, 0.002784020000000, 0.029105990000000, -0.021093010000000, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(4) << 0, 0.0100, 0, -0.000197010000000, 1.011092020000000, -0.000595010000000, -0.030394010000000, 0.002186020000000, 0.008407990000000, 0, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(5) << 0, 0, 0.0100, -0.000296010000000, -0.000595010000000, 1.010594020000000, 0.019306990000000, -0.011092010000000, 0.001388020000000, 0, 0, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    expected_prior_covariance_matrix.row(6) << 0, 0, 0, 0.002784020000000, -0.030394010000000, 0.019306990000000, 1.016178020000000, -0.001991010000000, -0.002390010000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(7) << 0, 0, 0, 0.029105990000000, 0.002186020000000, -0.011092010000000, -0.001991010000000, 1.015280020000000, -0.002989010000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(8) << 0, 0, 0,-0.021093010000000, 0.008407990000000, 0.001388020000000, -0.002390010000000, -0.002989010000000, 1.014182020000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0;

    expected_prior_covariance_matrix.row(9) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(10) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(11) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    expected_prior_covariance_matrix.row(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(13) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(14) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    expected_prior_covariance_matrix.row(15) << 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(16) << 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(17) << 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    expected_prior_covariance_matrix.row(18) << 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(19) << 0, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(20) << 0, 0, 0, 0, 0, 0, 0, 0, -0.0100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0, 0;

    expected_prior_covariance_matrix.row(21) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(22) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0, 0;
    expected_prior_covariance_matrix.row(23) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0, 0;

    expected_prior_covariance_matrix.row(24) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0, 0;
    expected_prior_covariance_matrix.row(25) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100, 0;
    expected_prior_covariance_matrix.row(26) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0100;

    Eigen::MatrixXd actual_prior_covariance_matrix = m_sensor_fusion->calculatePriorCovarianceMatrix(state);

    displayActualAndExpectedMatrices(expected_prior_covariance_matrix, actual_prior_covariance_matrix);
    ASSERT_EQ(expected_prior_covariance_matrix.rows(), actual_prior_covariance_matrix.rows());
    ASSERT_EQ(expected_prior_covariance_matrix.cols(), actual_prior_covariance_matrix.cols());
    ASSERT_TRUE(expected_prior_covariance_matrix.isApprox(actual_prior_covariance_matrix, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_calculate_innovation)
{
    setupHennieWithKoen();
    setupZeroSensorFusionNoise();
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());
    m_sensor_fusion->updateJointState(createEmptyJointStateForHennieWithKoen());
    EKFState state_prior;

    Eigen::VectorXd expected_innovation = Eigen::VectorXd::Zero(12);
    expected_innovation <<  0.38,
                            0.037,
                            -0.77,
                            0.38,
                            -0.049,
                            -0.77,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0;
    
    Eigen::VectorXd actual_innovation = m_sensor_fusion->calculateInnovation(state_prior);

    displayActualAndExpectedVectors(expected_innovation, actual_innovation);
    ASSERT_EQ(expected_innovation.rows(), actual_innovation.rows());
    ASSERT_TRUE(expected_innovation.isApprox(actual_innovation, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_calculate_observation_model)
{
    setupHennieWithKoen();
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());
    m_sensor_fusion->updateJointState(createEmptyJointStateForHennieWithKoen());
    EKFState state_prior;

    Eigen::MatrixXd expected_observation_model_matrix = Eigen::MatrixXd::Zero(12, 27);
    expected_observation_model_matrix.row(0) << -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_observation_model_matrix.row(1) << 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_observation_model_matrix.row(2) << 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_observation_model_matrix.row(3) << -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_observation_model_matrix.row(4) << 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_observation_model_matrix.row(5) << 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_observation_model_matrix.row(6) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0;
    expected_observation_model_matrix.row(7) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0;
    expected_observation_model_matrix.row(8) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0;
    expected_observation_model_matrix.row(9) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0;
    expected_observation_model_matrix.row(10) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0;
    expected_observation_model_matrix.row(11) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1;

    Eigen::MatrixXd actual_observation_model_matrix = m_sensor_fusion->getObservationModelMatrix(state_prior);

    displayActualAndExpectedMatrices(expected_observation_model_matrix, actual_observation_model_matrix);
    ASSERT_EQ(expected_observation_model_matrix.rows(), actual_observation_model_matrix.rows());
    ASSERT_EQ(expected_observation_model_matrix.cols(), actual_observation_model_matrix.cols());
    ASSERT_TRUE(expected_observation_model_matrix.isApprox(actual_observation_model_matrix, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_calculate_innovation_covariance_matrix)
{
    setupHennieWithKoen();
    setupZeroSensorFusionNoise();
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());
    m_sensor_fusion->updateJointState(createEmptyJointStateForHennieWithKoen());
    EKFState state_prior;

    Eigen::MatrixXd expected_innovation_covariance_matrix = Eigen::MatrixXd::Zero(12, 12);
    expected_innovation_covariance_matrix.row(0) << 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_innovation_covariance_matrix.row(1) << 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
    expected_innovation_covariance_matrix.row(2) << 0, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    expected_innovation_covariance_matrix.row(3) << 1, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_innovation_covariance_matrix.row(4) << 0, 1, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0;
    expected_innovation_covariance_matrix.row(5) << 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 0, 0;
    expected_innovation_covariance_matrix.row(6) << 0, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0;
    expected_innovation_covariance_matrix.row(7) << 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0;
    expected_innovation_covariance_matrix.row(8) << 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 1;
    expected_innovation_covariance_matrix.row(9) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 2, 0, 0;
    expected_innovation_covariance_matrix.row(10) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2, 0;
    expected_innovation_covariance_matrix.row(11) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2;

    Eigen::MatrixXd actual_innovation_covariance_matrix = m_sensor_fusion->calculateInnovationCovarianceMatrix(state_prior);

    displayActualAndExpectedMatrices(expected_innovation_covariance_matrix, actual_innovation_covariance_matrix);
    ASSERT_EQ(expected_innovation_covariance_matrix.rows(), actual_innovation_covariance_matrix.rows());
    ASSERT_EQ(expected_innovation_covariance_matrix.cols(), actual_innovation_covariance_matrix.cols());
    ASSERT_TRUE(expected_innovation_covariance_matrix.isApprox(actual_innovation_covariance_matrix, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_calculate_kalman_gain)
{
    setupHennieWithKoen();
    setupDummySensorFusionNoise();
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());
    m_sensor_fusion->updateJointState(createEmptyJointStateForHennieWithKoen());
    EKFState state_prior;

    Eigen::MatrixXd observation_model_matrix = Eigen::MatrixXd::Zero(12, 27);
    observation_model_matrix.row(0) << -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(1) << 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(2) << 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(3) << -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(4) << 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(5) << 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(6) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0;
    observation_model_matrix.row(7) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0;
    observation_model_matrix.row(8) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0;
    observation_model_matrix.row(9) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0;
    observation_model_matrix.row(10) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0;
    observation_model_matrix.row(11) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1;

    Eigen::MatrixXd expected_kalman_gain = Eigen::MatrixXd::Zero(27, 12);
    expected_kalman_gain.row(0) << -0.332225913621262, 0, 0, -0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(1) << 0, -0.332225913621262, 0, 0, -0.332225913621262, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(2) << 0, 0, -0.332225913621262, 0, 0, -0.332225913621262, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(3) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(5) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(6) << 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0, 0;
    expected_kalman_gain.row(7) << 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0;
    expected_kalman_gain.row(8) << 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262;
    expected_kalman_gain.row(9) << 0.661162461761126, 0, 0, -0.328936548139864, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(10) << 0, 0.661162461761126, 0, 0, -0.328936548139864, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(11) << 0, 0, 0.661162461761126, 0, 0, -0.328936548139864, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(12) << -0.328936548139864, 0, 0, 0.661162461761126, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(13) << 0, -0.328936548139864, 0, 0, 0.661162461761126, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(14) << 0, 0, -0.328936548139864, 0, 0, 0.661162461761126, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(15) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(16) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(17) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(18) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(19) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(20) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_kalman_gain.row(21) << 0, 0, 0, 0, 0, 0, -0.661162461761126, 0, 0, 0.328936548139864, 0, 0;
    expected_kalman_gain.row(22) << 0, 0, 0, 0, 0, 0, 0, -0.661162461761126, 0, 0, 0.328936548139864, 0;
    expected_kalman_gain.row(23) << 0, 0, 0, 0, 0, 0, 0, 0, -0.661162461761126, 0, 0, 0.328936548139864;
    expected_kalman_gain.row(24) << 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, -0.661162461761126, 0, 0;
    expected_kalman_gain.row(25) << 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, -0.661162461761126, 0;
    expected_kalman_gain.row(26) << 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, -0.661162461761126;

    Eigen::MatrixXd actual_kalman_gain = m_sensor_fusion->calculateKalmanGain(state_prior, observation_model_matrix);
    
    displayActualAndExpectedMatrices(expected_kalman_gain, actual_kalman_gain);
    ASSERT_EQ(expected_kalman_gain.rows(), actual_kalman_gain.rows());
    ASSERT_EQ(expected_kalman_gain.cols(), actual_kalman_gain.cols());
    ASSERT_TRUE(expected_kalman_gain.isApprox(actual_kalman_gain, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_compute_estimated_covariance_matrix)
{
    setupHennieWithKoen();
    setupZeroSensorFusionNoise();
    m_sensor_fusion->updateImu(std::make_shared<sensor_msgs::msg::Imu>());
    m_sensor_fusion->updateJointState(createEmptyJointStateForHennieWithKoen());
    EKFState state_prior;

    Eigen::MatrixXd observation_model_matrix = Eigen::MatrixXd::Zero(12, 27);
    observation_model_matrix.row(0) << -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(1) << 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(2) << 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(3) << -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(4) << 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(5) << 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    observation_model_matrix.row(6) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0;
    observation_model_matrix.row(7) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0;
    observation_model_matrix.row(8) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0;
    observation_model_matrix.row(9) << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0;
    observation_model_matrix.row(10) << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0;
    observation_model_matrix.row(11) << 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1;

    Eigen::MatrixXd kalman_gain = Eigen::MatrixXd::Zero(27, 12);
    kalman_gain.row(0) << -0.332225913621262, 0, 0, -0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(1) << 0, -0.332225913621262, 0, 0, -0.332225913621262, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(2) << 0, 0, -0.332225913621262, 0, 0, -0.332225913621262, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(3) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(5) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(6) << 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0, 0;
    kalman_gain.row(7) << 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0;
    kalman_gain.row(8) << 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262;
    kalman_gain.row(9) << 0.661162461761126, 0, 0, -0.328936548139864, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(10) << 0, 0.661162461761126, 0, 0, -0.328936548139864, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(11) << 0, 0, 0.661162461761126, 0, 0, -0.328936548139864, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(12) << -0.328936548139864, 0, 0, 0.661162461761126, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(13) << 0, -0.328936548139864, 0, 0, 0.661162461761126, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(14) << 0, 0, -0.328936548139864, 0, 0, 0.661162461761126, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(15) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(16) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(17) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(18) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(19) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(20) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    kalman_gain.row(21) << 0, 0, 0, 0, 0, 0, -0.661162461761126, 0, 0, 0.328936548139864, 0, 0;
    kalman_gain.row(22) << 0, 0, 0, 0, 0, 0, 0, -0.661162461761126, 0, 0, 0.328936548139864, 0;
    kalman_gain.row(23) << 0, 0, 0, 0, 0, 0, 0, 0, -0.661162461761126, 0, 0, 0.328936548139864;
    kalman_gain.row(24) << 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, -0.661162461761126, 0, 0;
    kalman_gain.row(25) << 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, -0.661162461761126, 0;
    kalman_gain.row(26) << 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, -0.661162461761126;

    Eigen::MatrixXd expected_estimated_covariance_matrix = Eigen::MatrixXd::Zero(27, 27);
    expected_estimated_covariance_matrix.row(0) << 0.335548172757475, 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(1) << 0, 0.335548172757475, 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(2) << 0, 0, 0.335548172757475, 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(3) << 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(4) << 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(5) << 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(6) << 0, 0, 0, 0, 0, 0, 0.335548172757475, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0, 0;
    expected_estimated_covariance_matrix.row(7) << 0, 0, 0, 0, 0, 0, 0, 0.335548172757475, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262, 0;
    expected_estimated_covariance_matrix.row(8) << 0, 0, 0, 0, 0, 0, 0, 0, 0.335548172757475, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0.332225913621262;
    expected_estimated_covariance_matrix.row(9) << 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0.338837538238874, 0, 0, 0.328936548139864, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(10) << 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0.338837538238874, 0, 0, 0.328936548139864, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(11) << 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0.338837538238874, 0, 0, 0.328936548139864, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(12) << 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, 0.338837538238874, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(13) << 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, 0.338837538238874, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(14) << 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, 0.338837538238874, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(15) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(16) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(17) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(18) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(19) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(20) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0;
    expected_estimated_covariance_matrix.row(21) << 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.338837538238874, 0, 0, 0.328936548139864, 0, 0;
    expected_estimated_covariance_matrix.row(22) << 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.338837538238874, 0, 0, 0.328936548139864, 0;
    expected_estimated_covariance_matrix.row(23) << 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.338837538238874, 0, 0, 0.328936548139864;
    expected_estimated_covariance_matrix.row(24) << 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, 0.338837538238874, 0, 0;
    expected_estimated_covariance_matrix.row(25) << 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, 0.338837538238874, 0;
    expected_estimated_covariance_matrix.row(26) << 0, 0, 0, 0, 0, 0, 0, 0, 0.332225913621262, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.328936548139864, 0, 0, 0.338837538238874;

    Eigen::MatrixXd actual_estimated_covariance_matrix = m_sensor_fusion->calculateEstimatedCovarianceMatrix(state_prior, kalman_gain, observation_model_matrix);

    displayActualAndExpectedMatrices(actual_estimated_covariance_matrix, expected_estimated_covariance_matrix);
    ASSERT_EQ(expected_estimated_covariance_matrix.rows(), actual_estimated_covariance_matrix.rows());
    ASSERT_EQ(expected_estimated_covariance_matrix.cols(), actual_estimated_covariance_matrix.cols());
    ASSERT_TRUE(expected_estimated_covariance_matrix.isApprox(actual_estimated_covariance_matrix, 1e-13));
}

TEST_F(SensorFusionTest, test_should_be_able_to_converge_under_ten_seconds_time_limit_with_noiseless_imu)
{
    testNoiselessKalmanFilterConvergenceForHennieWithKoen(1e-6, 10.0);
}

TEST_F(SensorFusionTest, test_should_be_able_to_converge_under_a_second_time_limit_with_noiseless_imu)
{
    testNoiselessKalmanFilterConvergenceForHennieWithKoen(1e-6, 1.0);
}

// TEST_F(SensorFusionTest, test_should_be_able_to_converge_x_iterations_under_x_seconds_time_limit_with_noisy_imu)
// {
//     testNoisyKalmanFilterConvergenceForHennieWithKoen(1e-6, 1.0, 10);
// }

/************************************************
 * 
 * Sensor Fusion Node Tests
 *
 ************************************************/

TEST_F(SensorFusionTest, test_should_get_com_for_hennie_with_koen)
{
    setupHennieWithKoen();
    sensor_msgs::msg::Imu::SharedPtr imu = std::make_shared<sensor_msgs::msg::Imu>();
    sensor_msgs::msg::JointState::SharedPtr joint_state = createEmptyJointStateForHennieWithKoen();

    m_sensor_fusion->updateImu(imu);
    m_sensor_fusion->updateJointState(joint_state);

    ASSERT_NO_FATAL_FAILURE(m_sensor_fusion->getCOM());
}

TEST_F(SensorFusionTest, test_should_get_zmp_for_hennie_with_koen)
{
    setupHennieWithKoen();
    sensor_msgs::msg::Imu::SharedPtr imu = std::make_shared<sensor_msgs::msg::Imu>();
    sensor_msgs::msg::JointState::SharedPtr joint_state = createEmptyJointStateForHennieWithKoen();
    geometry_msgs::msg::Point left_foot_position = createZeroPoint();
    geometry_msgs::msg::Point right_foot_position = createZeroPoint();

    m_sensor_fusion->updateImu(imu);
    m_sensor_fusion->updateJointState(joint_state);
    m_sensor_fusion->updateStanceLeg(&left_foot_position, &right_foot_position);

    ASSERT_NO_FATAL_FAILURE(m_sensor_fusion->getZMP());
}

TEST_F(SensorFusionTest, test_should_get_foot_poses_for_hennie_with_koen)
{
    setupHennieWithKoen();
    sensor_msgs::msg::JointState::SharedPtr joint_state = createEmptyJointStateForHennieWithKoen();
    ASSERT_NO_FATAL_FAILURE(m_sensor_fusion->getFootPoses());
}

TEST_F(SensorFusionTest, test_should_get_feet_contact_heights_for_hennie_with_koen)
{
    setupHennieWithKoen();
    sensor_msgs::msg::JointState::SharedPtr joint_state = createEmptyJointStateForHennieWithKoen();
    ASSERT_NO_FATAL_FAILURE(m_sensor_fusion->getFootContactHeight());
}

// NOLINTEND
#endif // __clang_analyzer__