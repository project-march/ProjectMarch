#include "state_estimator/zmp_estimator.hpp"

ZmpEstimator::ZmpEstimator()
{
}

void ZmpEstimator::set_zmp(CenterOfMass com, IMU imu, rclcpp::Time current_time, geometry_msgs::msg::TransformStamped T_imu_com)
{
    //obtain the necessary variables
    // This includes calculating imu acceleration :(
    tf2::Vector3 acceleration_imu(imu.data.linear_acceleration.x,imu.data.linear_acceleration.y,imu.data.linear_acceleration.z);
    tf2::Vector3 com_pos(com.position.point.x, com.position.point.y, com.position.point.z);
    tf2::Vector3 angular_velocity(imu.data.angular_velocity.x, imu.data.angular_velocity.y, imu.data.angular_velocity.z);
    double dt = (current_time-m_last_updated_time).nanoseconds()/1e9;
    tf2::Vector3 angular_acceleration = (angular_velocity-m_last_updated_angular_velocity)/dt;
    // There might be something wrong here, not sure we have to check this
    tf2::Vector3 imu_to_com(T_imu_com.transform.translation.x,T_imu_com.transform.translation.y,T_imu_com.transform.translation.z);

    
    tf2::Vector3 linear_acceleration_com = angular_velocity + angular_acceleration.cross(imu_to_com) - (angular_velocity*angular_velocity)*imu_to_com;
    //calculate the actual zmp
    double g = 9.81;
    m_position.header.frame_id = "map";
    m_position.point.x = com.position.point.x - linear_acceleration_com.getX()/g * linear_acceleration_com.getZ();
    m_position.point.y = com.position.point.y - linear_acceleration_com.getY()/g * linear_acceleration_com.getZ();
    m_position.point.z = 0.0;

    set_time(current_time);
}

geometry_msgs::msg::PointStamped ZmpEstimator::get_zmp()
{
    return m_position;
}

void ZmpEstimator::set_time(rclcpp::Time current_time){
    m_last_updated_time = current_time;
}

void ZmpEstimator::set_previous_angular_velocity(IMU imu)
{
    m_last_updated_angular_velocity = tf2::Vector3(imu.data.angular_velocity.x, imu.data.angular_velocity.y, imu.data.angular_velocity.z);
}