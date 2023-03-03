#include "com_estimator.hpp"
#include "cop_estimator.hpp"
#include "footstep_estimator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "imu_estimator.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
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

    std::vector<PressureSensor> get_pressure_sensors();

    geometry_msgs::msg::TransformStamped get_frame_transform(const std::string&, const std::string&);

private:
    void sensor_callback(sensor_msgs::msg::Imu::SharedPtr msg);

    void state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    void publish_robot_state();

    void publish_robot_frames();

    void initialize_imus();

    void publish_com_frame();

    void update_foot_frames();

    void visualize_joints();

    rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr m_state_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_sensor_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_state_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_com_pos_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_foot_pos_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_zmp_pos_publisher;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_rviz_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_joint_broadcaster;
    JointEstimator m_joint_estimator;
    ComEstimator m_com_estimator;
    CopEstimator m_cop_estimator;
    ImuEstimator m_imu_estimator;
    ZmpEstimator m_zmp_estimator;
    FootstepEstimator m_footstep_estimator;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_joint_listener;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif