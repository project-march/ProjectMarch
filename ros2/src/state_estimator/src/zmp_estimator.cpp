#include "state_estimator/zmp_estimator.hpp"

ZmpEstimator::ZmpEstimator()
{
}

void ZmpEstimator::set_zmp()
{
    // obtain the necessary variables
    // This includes calculating imu acceleration :(
    // double dt = (current_time - m_last_updated_time).nanoseconds() / 1e9;
    tf2::Vector3 linear_acceleration_com = get_com_acceleration();
    // calculate the actual zmp
    double g = 9.81;
    m_position.header.frame_id = "map";
    m_position.point.x = m_com_history[0].x - linear_acceleration_com.getX() / g * linear_acceleration_com.getZ();
    m_position.point.y = m_com_history[0].y - linear_acceleration_com.getY() / g * linear_acceleration_com.getZ();
    m_position.point.z = 0.0;

    // set_time(current_time);
}

geometry_msgs::msg::PointStamped ZmpEstimator::get_zmp()
{
    return m_position;
}

void ZmpEstimator::set_com_velocity()
{
    // We use a try to wait until the entire time has been filled

    // Here, we calculate the velocity immediately.
    // We do this to avoid doing this every time we want to set the zmp. Instead, we can just save old values
    try {
        double dt = (m_com_time_history[1] - m_com_time_history[2]).nanoseconds() / 1e9;
        m_com_velocity_history[1].x = (m_com_history[1].x - m_com_history[2].x) / dt;
        m_com_velocity_history[1].y = (m_com_history[1].y - m_com_history[2].y) / dt;
        m_com_velocity_history[1].z = (m_com_history[1].z - m_com_history[2].z) / dt;

        dt = (m_com_time_history[0] - m_com_time_history[1]).nanoseconds() / 1e9;
        m_com_velocity_history[0].x = (m_com_history[0].x - m_com_history[1].x) / dt;
        m_com_velocity_history[0].y = (m_com_history[0].y - m_com_history[1].y) / dt;
        m_com_velocity_history[0].z = (m_com_history[0].z - m_com_history[1].z) / dt;
    } catch (...) {
    }
}

tf2::Vector3 ZmpEstimator::get_com_acceleration()
{
    try {
        double dt = (m_com_time_history[0] - m_com_time_history[1]).nanoseconds() / 1e9;
        double acc_x = (m_com_velocity_history[0].x - m_com_velocity_history[1].x) / dt;
        double acc_y = (m_com_velocity_history[0].y - m_com_velocity_history[1].y) / dt;
        double acc_z = (m_com_velocity_history[0].z - m_com_velocity_history[1].z) / dt;
        return tf2::Vector3(acc_x, acc_y, acc_z);
    } catch (...) {
        return tf2::Vector3(std::nan("0"), std::nan("0"), std::nan("0"));
    }
}

void ZmpEstimator::set_com_states(CenterOfMass com, rclcpp::Time com_time)
{
    // We're doing this in a native way for efficiency and readability
    // Since we always know the size is going to be 3, this is fine.
    m_com_history[2] = m_com_history[1];
    m_com_history[1] = m_com_history[0];
    m_com_history[0] = com.position.point;

    // We also shift the times
    m_com_time_history[2] = m_com_time_history[1];
    m_com_time_history[1] = m_com_time_history[0];
    m_com_time_history[0] = com_time;

    set_com_velocity();
}