#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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

#ifndef JOINT_ESTIMATOR
#define JOINT_ESTIMATOR
enum Rotation { X, Y, Z };

struct JointContainer {
    std::string name;
    geometry_msgs::msg::TransformStamped frame;
    double com_x;
    double com_y;
    double com_z;
    double length_x;
    double length_y;
    double length_z;
    Rotation hinge_axis;
};

class StateEstimator;

class JointEstimator {
public:
    JointEstimator(StateEstimator* owner, sensor_msgs::msg::JointState);

    void set_joint_states(sensor_msgs::msg::JointState);
    void set_individual_joint_state(std::string, double);
    const std::vector<geometry_msgs::msg::TransformStamped> get_joint_frames();

private:
    // sensor_msgs::msg::JointState m_joint_states;
    StateEstimator* m_owner;
    std::unordered_map<std::string, std::string> m_joint_child_link_map;
    std::vector<JointContainer> m_joints;

    void initialize_joints(sensor_msgs::msg::JointState);
};

#endif