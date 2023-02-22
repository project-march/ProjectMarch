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

#ifndef COM_ESTIMATOR
#define COM_ESTIMATOR

class StateEstimator;

class ComEstimator {
public:
    ComEstimator();
    void set_com_state(std::vector<CenterOfMass>);
    CenterOfMass get_com_state();

private:
    // sensor_msgs::msg::JointState m_joint_states;
    StateEstimator* m_owner;
    CenterOfMass m_center_of_mass;
};

#endif