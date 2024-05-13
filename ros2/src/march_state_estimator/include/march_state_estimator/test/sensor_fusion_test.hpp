/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__SENSOR_FUSION_TEST_HPP_
#define MARCH_STATE_ESTIMATOR__SENSOR_FUSION_TEST_HPP_

#include <gtest/gtest.h>

#include "march_state_estimator/sensor_fusion.hpp"

class SensorFusionTest : public ::testing::Test {
public:
    SensorFusionTest() = default;
    ~SensorFusionTest() = default;

protected:
    void SetUp() override {
        m_sensor_fusion = std::make_unique<SensorFusion>();
    }

    void TearDown() override {
        m_sensor_fusion.reset();
    }

    void testComputeEulerAngles(Eigen::Quaterniond orientation, Eigen::Vector3d expected_euler_angles) {
        Eigen::Vector3d euler_angles = m_sensor_fusion->computeEulerAngles(orientation);
        for (long int i = 0; i < euler_angles.size(); i++) {
            ASSERT_NEAR(euler_angles[i], expected_euler_angles[i], 1e-6);
        }
    }

    void testComputeExponentialMap(Eigen::Vector3d vector, Eigen::Quaterniond expected_quaternion) {
        Eigen::Quaterniond quaternion = m_sensor_fusion->computeExponentialMap(vector);
        ASSERT_NEAR(quaternion.w(), expected_quaternion.w(), 1e-6);
        for (long int i = 0; i < quaternion.vec().size(); i++) {
            ASSERT_NEAR(quaternion.vec()[i], expected_quaternion.vec()[i], 1e-6);
        }
    }

private:
    std::unique_ptr<SensorFusion> m_sensor_fusion;
};

#endif  // MARCH_STATE_ESTIMATOR__SENSOR_FUSION_TEST_HPP_