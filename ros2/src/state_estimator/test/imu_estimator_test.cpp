//
// Created by rixt on 20-3-23.
//


#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2022 Project March.

#include "imu_estimator.hpp"
//#include "mocks/mock_state_estimator.hpp"
//#include "sensor_msgs/msg/joint_state.hpp"
//#include "state_estimator.hpp"
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class ImuEstimatorTest : public testing::Test {
protected:
    void SetUp() override
    {
        imu_estimator = std::make_unique<ImuEstimator>();
    }
    std::unique_ptr<ImuEstimator> imu_estimator;
};

TEST_F(ImuEstimatorTest, setIMUTest)
{
    IMU imu;
    imu.name = "test_imu";
    imu.base_frame = "test_base_frame";
    sensor_msgs::msg::Imu data;
    geometry_msgs::msg::Quaternion q;
    q.x = 3.0;
    q.y = 4.0;
    q.z = 5.0;
    q.w = 6.0;
    geometry_msgs::msg::Vector3 av;
    av.x = 1.0;
    av.y = 1.5;
    av.z = 2.0;
    geometry_msgs::msg::Vector3 la;
    la.x = 1.25;
    la.y = 1.55;
    la.z = 2.10;

    data.orientation = q;
    data.orientation_covariance = {1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0};
    data.angular_velocity = av;
    data.angular_velocity_covariance = {2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0};
    data.linear_acceleration = la;
    data.linear_acceleration_covariance = {3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0};

    imu.data = data;

    geometry_msgs::msg::Transform imu_location;
    geometry_msgs::msg::Quaternion rotation;
    rotation.x = 0.0;
    rotation.y = 2.0;
    rotation.z = 3.0;
    rotation.w = 5.0;
    geometry_msgs::msg::Vector3 translation;
    translation.x = 1.0;
    translation.y = 1.7;
    translation.z = 2.2;
    imu_location.rotation = rotation;
    imu_location.translation = translation;

    imu.imu_location = imu_location;

    imu_estimator->set_imu(imu);


    ASSERT_EQ(imu_estimator->get_imu(), imu);
}

TEST_F(ImuEstimatorTest, changeOrientationIMUTest)
{
    IMU imu;
    imu.name = "test_imu";
    imu.base_frame = "test_base_frame";
    sensor_msgs::msg::Imu data;
    geometry_msgs::msg::Quaternion q;
    q.x = 3.0;
    q.y = 4.0;
    q.z = 5.0;
    q.w = 6.0;
    geometry_msgs::msg::Vector3 av;
    av.x = 1.0;
    av.y = 1.5;
    av.z = 2.0;
    geometry_msgs::msg::Vector3 la;
    la.x = 1.25;
    la.y = 1.55;
    la.z = 2.10;

    data.orientation = q;
    data.orientation_covariance = {1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0};
    data.angular_velocity = av;
    data.angular_velocity_covariance = {2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0};
    data.linear_acceleration = la;
    data.linear_acceleration_covariance = {3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0};

    imu.data = data;

    geometry_msgs::msg::Transform imu_location;
    geometry_msgs::msg::Quaternion rotation;
    rotation.x = 0.0;
    rotation.y = 2.0;
    rotation.z = 3.0;
    rotation.w = 5.0;
    geometry_msgs::msg::Vector3 translation;
    translation.x = 1.0;
    translation.y = 1.7;
    translation.z = 2.2;
    imu_location.rotation = rotation;
    imu_location.translation = translation;

    imu.imu_location = imu_location;

    imu_estimator->set_imu(imu);

    imu.data.linear_acceleration.x = 3.0;


    ASSERT_FALSE(imu_estimator->get_imu() == imu);
}

TEST_F(ImuEstimatorTest, updateIMUTest)
{
    IMU imu;
    imu.name = "test_imu";
    imu.base_frame = "test_base_frame";
    sensor_msgs::msg::Imu data;
    geometry_msgs::msg::Quaternion q;
    q.x = 3.0;
    q.y = 4.0;
    q.z = 5.0;
    q.w = 6.0;
    geometry_msgs::msg::Vector3 av;
    av.x = 1.0;
    av.y = 1.5;
    av.z = 2.0;
    geometry_msgs::msg::Vector3 la;
    la.x = 1.25;
    la.y = 1.55;
    la.z = 2.10;

    data.orientation = q;
    data.orientation_covariance = {1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0};
    data.angular_velocity = av;
    data.angular_velocity_covariance = {2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0};
    data.linear_acceleration = la;
    data.linear_acceleration_covariance = {3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0};

    imu.data = data;

    geometry_msgs::msg::Transform imu_location;
    geometry_msgs::msg::Quaternion rotation;
    rotation.x = 0.0;
    rotation.y = 2.0;
    rotation.z = 3.0;
    rotation.w = 5.0;
    geometry_msgs::msg::Vector3 translation;
    translation.x = 1.0;
    translation.y = 1.7;
    translation.z = 2.2;
    imu_location.rotation = rotation;
    imu_location.translation = translation;

    imu.imu_location = imu_location;

    imu_estimator->set_imu(imu);

    sensor_msgs::msg::Imu data_updated;
    geometry_msgs::msg::Quaternion q_updated;
    q_updated.x = 3.5;
    q_updated.y = 4.0;
    q_updated.z = 5.5;
    q_updated.w = 6.0;
    geometry_msgs::msg::Vector3 av_updated;
    av_updated.x = 1.5;
    av_updated.y = 1.5;
    av_updated.z = 2.0;
    geometry_msgs::msg::Vector3 la_updated;
    la_updated.x = 1.25;
    la_updated.y = 1.55;
    la_updated.z = 2.10;

    data_updated.orientation = q_updated;
    data_updated.orientation_covariance = {1.0, 2.5, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0, 3.0};
    data_updated.angular_velocity = av_updated;
    data_updated.angular_velocity_covariance = {2.0, 3.0, 1.0, 2.0, 3.5, 1.0, 2.0, 3.0, 1.0};
    data_updated.linear_acceleration = la_updated;
    data_updated.linear_acceleration_covariance = {3.0, 1.5, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0};

    IMU imu_copy = imu;
//    imu_copy.name = "test_imu";
//    imu_copy.base_frame = "test_base_frame";
    imu_copy.data = data_updated;
//    imu_copy.imu_location = imu_location;

    ASSERT_FALSE(imu_estimator->get_imu() == imu_copy);

    imu_estimator->update_imu(data_updated);


    ASSERT_EQ(imu_estimator->get_imu(), imu_copy);
}



// NOLINTEND
#endif