#include "state_estimator/imu_estimator.hpp"

ImuEstimator::ImuEstimator()
{
}

void ImuEstimator::set_imu(IMU& imu_to_set, int index)
{
    m_imu[index] = imu_to_set;
}

void ImuEstimator::update_imu(sensor_msgs::msg::Imu new_data, int index)
{
    m_imu[index].data = new_data;
    m_imu[index].data.header.frame_id = "map";
}

IMU& ImuEstimator::get_imu(int index)
{
    return m_imu[index];
}