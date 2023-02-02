#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"

class StateEstimator;

class JointEstimator {
public:
    JointEstimator(StateEstimator* owner, sensor_msgs::msg::JointState initial_joint_states);

    void set_joint_states(sensor_msgs::msg::JointState& new_joint_states);
    sensor_msgs::msg::JointState get_joint_states();

private:
    sensor_msgs::msg::JointState m_joint_states;
    const StateEstimator* m_owner;

    void get_joint_frames();
};