#include "geometry_msgs/msg/point.hpp"
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

struct CenterOfMass {
    geometry_msgs::msg::PointStamped position;
    double mass;
    friend bool operator==(CenterOfMass a, CenterOfMass b)
    {
        return (a.position.point.x == b.position.point.x && a.position.point.y == b.position.point.y
            && a.position.point.z == b.position.point.z && abs(a.mass - b.mass) < 1e-6);
    };
};

struct JointContainer {
    std::string name;
    std::string base_link_name;
    double axis;
    geometry_msgs::msg::TransformStamped frame;
    CenterOfMass com;
    double length_x;
    double length_y;
    double length_z;
    Rotation hinge_axis;
    // Add this in for Testing
    friend bool operator==(JointContainer a, JointContainer b)
    {
        return (abs(a.length_x - b.length_x) < 1e-6 && abs(a.length_y - b.length_y) < 1e-6
            && abs(a.length_z - b.length_z) < 1e-6 && abs(a.length_z - b.length_z) < 1e-6
            && (a.hinge_axis == b.hinge_axis) && (a.com == b.com));
    };
};

class StateEstimator;

class JointEstimator {
public:
    JointEstimator(StateEstimator* owner);

    const JointContainer get_individual_joint(std::string);
    void set_joint_states(sensor_msgs::msg::JointState::SharedPtr);
    void set_individual_joint_state(std::string, double);
    const std::vector<JointContainer> get_joints();
    const std::vector<geometry_msgs::msg::TransformStamped> get_joint_frames();
    std::vector<CenterOfMass> get_joint_com_positions(std::string);
    std::vector<double> get_feet_height();
    std::vector<std::array<double, 3>> transformFeetPositionsToExoFrame() const; 

private:
    // sensor_msgs::msg::JointState m_joint_states;
    StateEstimator* m_owner;
    std::unordered_map<std::string, std::string> m_joint_child_link_map;
    std::vector<JointContainer> m_joints;

    std::unordered_map<std::string, std::string> interpret_joint_links();
    void initialize_joints();
};

#endif