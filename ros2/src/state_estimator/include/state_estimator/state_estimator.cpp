#include "state_estimator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

StateEstimator::StateEstimator()
    : Node("state_estimator_node")
    , m_joint_estimator(this, get_initial_joint_states())
    , m_com_estimator(this)
{
    m_state_publisher = this->create_publisher<march_shared_msgs::msg::RobotState>("robot_state", 10);
    m_sensor_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&StateEstimator::sensor_callback, this, _1));
    m_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10, std::bind(&StateEstimator::state_callback, this, _1));
    m_joint_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(1000ms, std::bind(&StateEstimator::publish_robot_frames, this));
}

sensor_msgs::msg::JointState StateEstimator::get_initial_joint_states()
{
    sensor_msgs::msg::JointState initial_joint_state;
    // change it so the names are obtained from the parameter
    initial_joint_state.name = { "right_ankle", "right_knee", "right_hip_fe", "right_hip_aa", "left_ankle", "left_knee",
        "left_hip_fe", "left_hip_aa", "right_origin", "left_origin" };
    initial_joint_state.position = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    return initial_joint_state;
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

void StateEstimator::publish_robot_frames()
{
    m_joint_estimator.set_individual_joint_state("right_ankle", 3.14);
    RCLCPP_INFO(this->get_logger(), "Still running...");
    for (auto i : m_joint_estimator.get_joint_frames()) {
        m_joint_tf_broadcaster->sendTransform(i);
    }
}