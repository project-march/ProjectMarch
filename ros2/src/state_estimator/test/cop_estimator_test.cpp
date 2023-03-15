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
        mock_sensor.name = "l_mock_sensor";
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
    mock_sensor.name = "l_mock_sensor";
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

TEST_F(CopEstimatorTest, setCopMoreSensorsTest)
{
    CenterOfPressure expected_cop;
    expected_cop.position.point.x = 1;
    expected_cop.position.point.y = 1;
    expected_cop.position.point.z = 0;
    expected_cop.pressure = 6;
    std::vector<PressureSensor> sensors;
    PressureSensor mock_sensor1;
    mock_sensor1.name = "l_mock_sensor1";
    CenterOfPressure cop;
    cop.position.point.x = 0;
    cop.position.point.y = 0;
    cop.position.point.z = 0;
    cop.pressure = 3;
    mock_sensor1.centre_of_pressure = cop;
    sensors.push_back(mock_sensor1);
    PressureSensor mock_sensor2;
    mock_sensor1.name = "l_mock_sensor2";
    cop.position.point.x = 3;
    cop.position.point.y = 0;
    cop.position.point.z = 0;
    cop.pressure = 1;
    mock_sensor2.centre_of_pressure = cop;
    sensors.push_back(mock_sensor2);
    PressureSensor mock_sensor3;
    mock_sensor3.name = "l_mock_sensor3";
    cop.position.point.x = 3;
    cop.position.point.y = 3;
    cop.position.point.z = 0;
    cop.pressure = 1;
    mock_sensor3.centre_of_pressure = cop;
    sensors.push_back(mock_sensor3);
    PressureSensor mock_sensor4;
    mock_sensor4.name = "l_mock_sensor4";
    cop.position.point.x = 0;
    cop.position.point.y = 3;
    cop.position.point.z = 0;
    cop.pressure = 1;
    mock_sensor4.centre_of_pressure = cop;
    sensors.push_back(mock_sensor4);
    cop_estimator->set_cop_state(sensors);
    ASSERT_EQ(this->cop_estimator->get_cop_state(), expected_cop);
}

TEST_F(CopEstimatorTest, setZeroPressureCopTest)
{
    std::vector<PressureSensor> sensors;
    PressureSensor mock_sensor;
    mock_sensor.name = "l_mock_sensor";
    CenterOfPressure cop;
    cop.position.point.x = 0;
    cop.position.point.y = 0;
    cop.position.point.z = 0;
    cop.pressure = 0;
    mock_sensor.centre_of_pressure = cop;
    sensors.push_back(mock_sensor);
    cop_estimator->set_cop_state(sensors);
    double expected = 0.0;
    ASSERT_EQ(cop_estimator->get_cop_state().pressure, expected);
}

TEST_F(CopEstimatorTest, testUpdateSensorPressure)
{
    double pressure = 2;
    std::map<std::string, double> update_map = { { "l_mock_sensor", pressure } };
    cop_estimator->update_sensor_pressures(update_map);
    auto updated_sensors = cop_estimator->get_sensors();
    for (auto sensor : updated_sensors) {
        ASSERT_EQ(sensor.centre_of_pressure.pressure, pressure);
    }
}

// NOLINTEND
#endif