#include "state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

StateEstimator::StateEstimator()
    : Node("state_estimator_node")
    , m_joint_estimator(JointEstimator(this, sensor_msgs::msg::JointState()))
{
    m_state_publisher = this->create_publisher<march_shared_msgs::msg::RobotState>("robot_state", 10);
    m_sensor_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&StateEstimator::sensor_callback, this, _1));
    m_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10, std::bind(&StateEstimator::state_callback, this, _1));
}

void StateEstimator::sensor_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{

}

void StateEstimator::state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
{

}

void StateEstimator::publish_robot_state()
{
    auto msg = march_shared_msgs::msg::RobotState();
    msg.stamp = this->get_clock()->now();
    msg.joint_names.push_back("");
    msg.joint_pos.push_back(0);
    msg.joint_vel.push_back(0);
    msg.sensor_names.push_back("");
    msg.sensor_data.push_back(0);

    m_state_publisher->publish(msg);
}
