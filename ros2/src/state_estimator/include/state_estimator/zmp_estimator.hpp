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
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_map>
#include "cop_estimator.hpp"
#ifndef ZMP_ESTIMATOR
#define ZMP_ESTIMATOR


class ZmpEstimator {
public:
    ZmpEstimator();
    void set_zmp(CenterOfMass, CenterOfPressure);
    geometry_msgs::msg::PointStamped get_zmp();
    

private:
    geometry_msgs::msg::PointStamped m_position;
};

#endif