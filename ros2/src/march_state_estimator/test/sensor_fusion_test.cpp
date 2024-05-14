/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/test/sensor_fusion_test.hpp"

#include <random>
#include <iostream>

/***********************************************************************************
 * SensorFusion::computeEulerAngles() tests
 ***********************************************************************************/

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_identity_orientation_as_zeros) {
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(0.0, 0.0, 0.0);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_0_degrees_roll_pitch_yaw_orientation) {
    Eigen::Quaterniond orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                                         Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                                         Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(0.0, 0.0, 0.0);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_90_degrees_roll_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(M_PI_2, 0.0, 0.0);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_90_degrees_pitch_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(0.0, M_PI_2, 0.0);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_90_degrees_yaw_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(0.0, 0.0, M_PI_2);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_90_degrees_roll_pitch_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(M_PI_2, M_PI_2, 0.0);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_90_degrees_roll_yaw_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(M_PI_2, 0.0, M_PI_2);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_90_degrees_pitch_yaw_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(0.0, M_PI_2, M_PI_2);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_180_degrees_roll_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(M_PI, 0.0, 0.0);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_180_degrees_pitch_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(0.0, M_PI, 0.0);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_180_degrees_yaw_orientation) {
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(0.0, 0.0, M_PI);
    testComputeEulerAngles(orientation, expected_euler_angles);
}

TEST_F(SensorFusionTest, test_should_compute_euler_angles_from_random_orientation_for_1000_iterations) {
    const unsigned int iterations = 1000;
    for (unsigned int i = 0; i < iterations; i++) {
        std::random_device rd;

        // Generate random values for each quaternion component in the range [-pi, pi]
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis_roll(0, M_PI);
        std::uniform_real_distribution<double> dis(-M_PI, M_PI);

        const double angle_roll = dis_roll(gen);
        const double angle_pitch = dis(gen);
        const double angle_yaw = dis(gen);

        Eigen::Quaterniond orientation(Eigen::AngleAxisd(angle_roll, Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(angle_pitch, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(angle_yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Vector3d expected_euler_angles = Eigen::Vector3d(angle_roll, angle_pitch, angle_yaw);
        testComputeEulerAngles(orientation, expected_euler_angles);
    }
}

/***********************************************************************************
 * SensorFusion::computeExponentialMap() tests
 ***********************************************************************************/

TEST_F(SensorFusionTest, test_should_compute_exponential_map_from_zero_vector_as_identity_quaternion) {
    Eigen::Vector3d zero_vector = Eigen::Vector3d::Zero();
    Eigen::Quaterniond expected_quaternion = Eigen::Quaterniond::Identity();
    testComputeExponentialMap(zero_vector, expected_quaternion);
}

TEST_F(SensorFusionTest, test_should_compute_exponential_map_from_1_0_0_vector_as_08775_04794_0_0_quaternion) {
    Eigen::Vector3d vector = Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Quaterniond expected_quaternion = Eigen::Quaterniond(
        0.8775825618903727161162815826038296519916451971097440529976108683, 
        0.4794255386042030002732879352155713880818033679406006751886166131,
        0.0, 
        0.0
    );
    testComputeExponentialMap(vector, expected_quaternion);
}

TEST_F(SensorFusionTest, test_should_compute_exponential_map_from_0_1_0_vector_as_04794_0_08775_0_quaternion) {
    Eigen::Vector3d vector = Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Quaterniond expected_quaternion = Eigen::Quaterniond(
        0.8775825618903727161162815826038296519916451971097440529976108683,
        0.0,
        0.4794255386042030002732879352155713880818033679406006751886166131, 
        0.0
    );
    testComputeExponentialMap(vector, expected_quaternion);
}

TEST_F(SensorFusionTest, test_should_compute_exponential_map_from_0_0_1_vector_as_04794_0_0_08775_quaternion) {
    Eigen::Vector3d vector = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Quaterniond expected_quaternion = Eigen::Quaterniond(
        0.8775825618903727161162815826038296519916451971097440529976108683,
        0.0,
        0.0,
        0.4794255386042030002732879352155713880818033679406006751886166131
    );
    testComputeExponentialMap(vector, expected_quaternion);
}

TEST_F(SensorFusionTest, test_should_compute_exponential_map_from_1_1_0_vector_as_05403_04593_04593_0_quaternion) {
    Eigen::Vector3d vector = Eigen::Vector3d(1.0, 1.0, 0.0);
    Eigen::Quaterniond expected_quaternion = Eigen::Quaterniond(
        0.7602445970756301512535471986232743946175831711351312085680,
        0.45936268493278426,
        0.45936268493278426,
        0.0
    );
    testComputeExponentialMap(vector, expected_quaternion);
}

/***********************************************************************************
 * SensorFusion::convertQuaternionToRotationMatrix() tests
 ***********************************************************************************/



/***********************************************************************************
 * SensorFusion::predictState() tests
 ***********************************************************************************/