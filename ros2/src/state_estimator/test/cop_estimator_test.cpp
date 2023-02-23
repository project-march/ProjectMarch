//
// Created by marco on 14-2-23.
//
#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.

#include "cop_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class CopEstimatorTest : public testing::Test {
protected:
    void SetUp() override
    {
        // Read the pressure sensors from the hardware interface
        PressureSensor mock_sensor;
        mock_sensor.name = "mock_sensor";
        CenterOfPressure cop;
        cop.position.point.x = 0;
        cop.position.point.y = 0;
        cop.position.point.z = 0;
        cop.pressure = 1;
        mock_sensor.centre_of_pressure = cop;
        sensors.push_back(mock_sensor);
        cop_estimator = std::make_unique<CopEstimator>(sensors);
    }
    std::unique_ptr<CopEstimator> cop_estimator;
    std::vector<PressureSensor> sensors;
};

TEST_F(CopEstimatorTest, setCopTest)
{
    CenterOfPressure expected_cop;
    expected_cop.position.point.x = 0;
    expected_cop.position.point.y = 0;
    expected_cop.position.point.z = 0;
    expected_cop.pressure = 0;
    CenterOfPressure actual_cop = this->cop_estimator->get_cop_state();
    ASSERT_EQ(actual_cop.pressure, expected_cop.pressure);
    std::vector<PressureSensor> sensors;
    PressureSensor mock_sensor;
    mock_sensor.name = "mock_sensor";
    CenterOfPressure cop;
    cop.position.point.x = 0;
    cop.position.point.y = 0;
    cop.position.point.z = 0;
    cop.pressure = 2;
    mock_sensor.centre_of_pressure = cop;
    sensors.push_back(mock_sensor);
    cop_estimator->set_cop_state(sensors);

    expected_cop.pressure = 2;
    ASSERT_EQ(this->cop_estimator->get_cop_state(), expected_cop);
}

TEST_F(CopEstimatorTest, setZeroPressureCopTest)
{
    std::vector<PressureSensor> sensors;
    PressureSensor mock_sensor;
    mock_sensor.name = "mock_sensor";
    CenterOfPressure cop;
    cop.position.point.x = 0;
    cop.position.point.y = 0;
    cop.position.point.z = 0;
    cop.pressure = 0;
    mock_sensor.centre_of_pressure = cop;
    sensors.push_back(mock_sensor);
    ASSERT_THROW(cop_estimator->set_cop_state(sensors), std::runtime_error);
}

TEST_F(CopEstimatorTest, testUpdateSensorPressure)
{
    double pressure = 2;
    std::map<std::string, double> update_map = { { "mock_sensor", pressure } };
    cop_estimator->update_sensor_pressures(update_map);
    auto updated_sensors = cop_estimator->get_sensors();
    for (auto sensor : updated_sensors) {
        ASSERT_EQ(sensor.centre_of_pressure.pressure, pressure);
    }
}

// NOLINTEND
#endif