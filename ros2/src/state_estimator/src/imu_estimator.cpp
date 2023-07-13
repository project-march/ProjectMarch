#include "state_estimator/imu_estimator.hpp"

ImuEstimator::ImuEstimator()
{
}

/**
 * Set the imu in the imu array.
 * @param imu_to_set Which IMU to update
 * @param index The index of the imu, Lower = 0, Upper is 1
 */
void ImuEstimator::set_imu(IMU& imu_to_set, int index)
{
    m_imu[index] = imu_to_set;
}

/**
 * Set the data of the specified IMU
 * @param new_data The dnew data to update teh IMU with
 * @param index The index of the imu, Lower = 0, Upper is 1
 */
void ImuEstimator::update_imu(sensor_msgs::msg::Imu new_data, int index)
{
    m_imu[index].data = new_data;
    m_imu[index].data.header.frame_id = "map";
}

/**
 *
 * @param index The index of the imu, Lower = 0, Upper is 1
 * @return The IMU.
 */
IMU& ImuEstimator::get_imu(int index)
{
    return m_imu[index];
}