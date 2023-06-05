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
        auto mock_sensor = new PressureSensor();
        mock_sensor->name = "l_mock_sensor";
        PressureSensor sensor;
        mock_sensor->position.point.x = 0;
        mock_sensor->position.point.y = 0;
        mock_sensor->position.point.z = 0;
        mock_sensor->pressure = 1;
        sensors.push_back(mock_sensor);
        cop_estimator = std::make_unique<CopEstimator>(sensors);
    }
    std::unique_ptr<CopEstimator> cop_estimator;
    std::vector<PressureSensor*> sensors;
};

TEST_F(CopEstimatorTest, setCopTest)
{
    geometry_msgs::msg::PointStamped expected_cop;
    expected_cop.point.x = 0;
    expected_cop.point.y = 0;
    expected_cop.point.z = 0;
    geometry_msgs::msg::PointStamped actual_cop = this->cop_estimator->get_cop();
    std::vector<PressureSensor*> sensors;
    auto* mock_sensor = new PressureSensor();
    mock_sensor->name = "l_mock_sensor";
    mock_sensor->position.point.x = 0;
    mock_sensor->position.point.y = 0;
    mock_sensor->position.point.z = 0;
    sensors.push_back(mock_sensor);

    geometry_msgs::msg::TransformStamped mock_transform;
    mock_transform.transform.translation.x = 0.0;
    mock_transform.transform.translation.y = 0.0;
    mock_transform.transform.translation.z = 0.0;
    mock_transform.transform.rotation.x = 0.0;
    mock_transform.transform.rotation.y = 0.0;
    mock_transform.transform.rotation.z = 0.0;
    mock_transform.transform.rotation.w = 1.0;
    cop_estimator->set_cop(sensors, { mock_transform, mock_transform });

    ASSERT_EQ(this->cop_estimator->get_cop(), expected_cop);
}

TEST_F(CopEstimatorTest, setCopMoreSensorsTest)
{
    geometry_msgs::msg::PointStamped expected_cop;
    expected_cop.point.x = 1;
    expected_cop.point.y = 1;
    expected_cop.point.z = 0;
    std::vector<PressureSensor*> sensors;
    auto* mock_sensor1 = new PressureSensor();
    mock_sensor1->name = "l_mock_sensor1";
    mock_sensor1->position.point.x = 0;
    mock_sensor1->position.point.y = 0;
    mock_sensor1->position.point.z = 0;
    mock_sensor1->pressure = 3;
    sensors.push_back(mock_sensor1);
    auto* mock_sensor2 = new PressureSensor();
    mock_sensor1->name = "l_mock_sensor2";
    mock_sensor2->position.point.x = 3;
    mock_sensor2->position.point.y = 0;
    mock_sensor2->position.point.z = 0;
    mock_sensor2->pressure = 1;
    sensors.push_back(mock_sensor2);
    auto* mock_sensor3 = new PressureSensor();
    mock_sensor3->name = "l_mock_sensor3";
    mock_sensor3->position.point.x = 3;
    mock_sensor3->position.point.y = 3;
    mock_sensor3->position.point.z = 0;
    mock_sensor3->pressure = 1;
    sensors.push_back(mock_sensor3);
    auto* mock_sensor4 = new PressureSensor();
    mock_sensor4->name = "l_mock_sensor4";
    mock_sensor4->position.point.x = 0;
    mock_sensor4->position.point.y = 3;
    mock_sensor4->position.point.z = 0;
    mock_sensor4->pressure = 1;
    sensors.push_back(mock_sensor4);

    geometry_msgs::msg::TransformStamped mock_transform;
    mock_transform.transform.translation.x = 0.0;
    mock_transform.transform.translation.y = 0.0;
    mock_transform.transform.translation.z = 0.0;
    mock_transform.transform.rotation.x = 0.0;
    mock_transform.transform.rotation.y = 0.0;
    mock_transform.transform.rotation.z = 0.0;
    mock_transform.transform.rotation.w = 1.0;

    cop_estimator->set_cop(sensors, { mock_transform, mock_transform });
    ASSERT_EQ(cop_estimator->get_cop(), expected_cop);
}

// TEST_F(CopEstimatorTest, setZeroPressureCopTest)
//{
//    std::vector<PressureSensor*> sensors;
//    PressureSensor* mock_sensor;
//    mock_sensor.name = "l_mock_sensor";
//    mock_sensor.position.point.x = 0;
//    mock_sensor.position.point.y = 0;
//    mock_sensor.position.point.z = 0;
//    mock_sensor.pressure = 0;
//    sensors.push_back(mock_sensor);
//
//    geometry_msgs::msg::TransformStamped mock_transform;
//    mock_transform.transform.translation.x = 0.0;
//    mock_transform.transform.translation.y = 0.0;
//    mock_transform.transform.translation.z = 0.0;
//    mock_transform.transform.rotation.x = 0.0;
//    mock_transform.transform.rotation.y = 0.0;
//    mock_transform.transform.rotation.z = 0.0;
//    mock_transform.transform.rotation.w = 1.0;
//
//    cop_estimator->set_cop(sensors, { mock_transform, mock_transform });
//    double expected = 0.0;
//    ASSERT_EQ(cop_estimator->get_cop(), expected);
//}

TEST_F(CopEstimatorTest, testUpdateSensorPressure)
{
    double pressure = 2;
    std::map<std::string, double> update_map = { { "l_mock_sensor", pressure } };
    cop_estimator->update_pressure_sensors(update_map);
    auto updated_sensors = cop_estimator->get_sensors();
    for (auto sensor : *updated_sensors) {
        ASSERT_EQ(sensor->pressure, pressure);
    }
}

// NOLINTEND
#endif