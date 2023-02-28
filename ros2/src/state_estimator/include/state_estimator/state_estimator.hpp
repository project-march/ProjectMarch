#include "com_estimator.hpp"
#include "cop_estimator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "imu_estimator.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "zmp_estimator.hpp"
#include <array>
#include <chrono>
#include <cstdio>
#include <string>

using std::placeholders::_1;
#ifndef STATE_ESTIMATOR
#define STATE_ESTIMATOR

class StateEstimator : public rclcpp::Node {
public:
    StateEstimator();

    sensor_msgs::msg::JointState get_initial_joint_states();

    std::map<std::string, double> update_pressure_sensors_data(
        std::vector<std::string> names, std::vector<double> pressure_values);

    geometry_msgs::msg::TransformStamped get_frame_transform(const std::string&, const std::string&);

    geometry_msgs::msg::Point transform_point(std::string&, std::string&, geometry_msgs::msg::Point&);

private:
    void sensor_callback(sensor_msgs::msg::Imu::SharedPtr msg);

    void state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    void pressure_sole_callback(march_shared_msgs::msg::PressureSolesData::SharedPtr msg);

    std::vector<PressureSensor> create_pressure_sensors();

    void publish_robot_state();

    void publish_robot_frames();

    void initialize_imus();

    void publish_com_frame();

    void update_foot_frames();

    rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr m_state_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_upper_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_lower_imu_subscriber;
    rclcpp::Subscription<march_shared_msgs::msg::PressureSolesData>::SharedPtr m_pressure_sole_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_state_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_com_pos_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_joint_broadcaster;
    JointEstimator m_joint_estimator;
    ComEstimator m_com_estimator;
    CopEstimator m_cop_estimator;
    ImuEstimator m_imu_estimator;
    ZmpEstimator m_zmp_estimator;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_joint_listener;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif