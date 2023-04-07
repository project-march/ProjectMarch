#include "cop_estimator.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <unordered_map>

#ifndef IMU_ESTIMATOR
#define IMU_ESTIMATOR

struct IMU {
    std::string name;
    std::string base_frame;
    sensor_msgs::msg::Imu data;
    geometry_msgs::msg::Transform imu_location;

    friend bool operator==(IMU a, IMU b)
    {
        return (a.name == b.name && a.base_frame == b.base_frame && a.data.orientation.x == b.data.orientation.x
            && a.data.orientation.y == b.data.orientation.y && a.data.orientation.z == b.data.orientation.z
            && a.data.orientation.w == b.data.orientation.w && a.data.angular_velocity.x == b.data.angular_velocity.x
            && a.data.angular_velocity.y == b.data.angular_velocity.y
            && a.data.angular_velocity.z == b.data.angular_velocity.z
            && a.data.linear_acceleration.x == b.data.linear_acceleration.x
            && a.data.linear_acceleration.y == b.data.linear_acceleration.y
            && a.data.linear_acceleration.z == b.data.linear_acceleration.z
            && a.data.orientation_covariance == b.data.orientation_covariance
            && a.data.angular_velocity_covariance == b.data.angular_velocity_covariance
            && a.data.linear_acceleration_covariance == b.data.linear_acceleration_covariance
            && a.imu_location.rotation.x == b.imu_location.rotation.x
            && a.imu_location.rotation.y == b.imu_location.rotation.y
            && a.imu_location.rotation.z == b.imu_location.rotation.z
            && a.imu_location.rotation.w == b.imu_location.rotation.w
            && a.imu_location.translation.x == b.imu_location.translation.x
            && a.imu_location.translation.y == b.imu_location.translation.y
            && a.imu_location.translation.z == b.imu_location.translation.z);
    }

public:
    geometry_msgs::msg::TransformStamped to_transform()
    {
        geometry_msgs::msg::TransformStamped transform_tosend;
        transform_tosend.header.frame_id = name;
        transform_tosend.child_frame_id = base_frame;
        transform_tosend.transform = imu_location;
        return geometry_msgs::msg::TransformStamped();
    };

    geometry_msgs::msg::TransformStamped get_imu_rotation()
    {
        geometry_msgs::msg::TransformStamped transform_tosend;
        transform_tosend.header = data.header;
        transform_tosend.header.frame_id = "map";
        transform_tosend.child_frame_id = "lowerIMU";
        transform_tosend.transform.rotation = data.orientation;
        // TODO: change this to relative IMU position on body
        transform_tosend.transform.translation.x = imu_location.translation.x;
        transform_tosend.transform.translation.y = imu_location.translation.y;
        transform_tosend.transform.translation.z = imu_location.translation.z;
        return transform_tosend;
    };
};

class ImuEstimator {
public:
    ImuEstimator();
    void set_imu(IMU&);
    void update_imu(sensor_msgs::msg::Imu);
    IMU& get_imu();

private:
    IMU m_imu;
};

#endif