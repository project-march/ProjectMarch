#include "state_estimator/imu_estimator.hpp"

ImuEstimator::ImuEstimator()
{
}

void ImuEstimator::set_imu(IMU& imu_to_set)
{
    m_imu = imu_to_set;
}

void ImuEstimator::update_imu(sensor_msgs::msg::Imu new_data)
{
    m_imu.data = new_data;
}

IMU& ImuEstimator::get_imu()
{
    return m_imu;
}