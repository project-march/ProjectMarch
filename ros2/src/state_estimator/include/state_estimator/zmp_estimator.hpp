#include "cop_estimator.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "imu_estimator.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <unordered_map>
#ifndef ZMP_ESTIMATOR
#define ZMP_ESTIMATOR

class ZmpEstimator {
public:
    ZmpEstimator();
    void set_zmp(CenterOfMass, IMU, rclcpp::Time, geometry_msgs::msg::TransformStamped);
    geometry_msgs::msg::PointStamped get_zmp();
    void set_time(rclcpp::Time);
    void set_previous_angular_velocity(IMU);

private:
    geometry_msgs::msg::PointStamped m_position;
    rclcpp::Time m_last_updated_time;
    tf2::Vector3 m_last_updated_angular_velocity;
};

#endif