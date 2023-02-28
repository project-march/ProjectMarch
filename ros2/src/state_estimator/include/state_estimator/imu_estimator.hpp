#include "cop_estimator.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include "geometry_msgs/msg/vector3.hpp"
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
        transform_tosend.transform.translation.x = 0.0;
        transform_tosend.transform.translation.y = 0.0;
        transform_tosend.transform.translation.z = 1.0;
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