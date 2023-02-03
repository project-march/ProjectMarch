#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    JointEstimator(StateEstimator*, sensor_msgs::msg::JointState);

    void set_joint_states(sensor_msgs::msg::JointState&);
    sensor_msgs::msg::JointState get_joint_states();

private:
    sensor_msgs::msg::JointState m_joint_states;
    const StateEstimator* m_owner;

    const std::vector<geometry_msgs::msg::TransformStamped> get_joint_frames();

    std::unordered_map<std::string, std::string> m_joint_child_link_map;
    std::vector<JointContainer> m_joints;

    void initialize_joints(sensor_msgs::msg::JointState);
};