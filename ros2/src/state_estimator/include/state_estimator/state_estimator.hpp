#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "imu_estimator.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "march_shared_msgs/msg/feet.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include <array>
#include <chrono>
#include <cstdio>
#include <string>

#include "state_estimator/exo_estimator.hpp"

using std::placeholders::_1;
#ifndef STATE_ESTIMATOR
#define STATE_ESTIMATOR

class StateEstimator : public rclcpp::Node {
public:
    StateEstimator();

    sensor_msgs::msg::JointState get_initial_joint_states();
    geometry_msgs::msg::TransformStamped get_frame_transform(const std::string&, const std::string&);

private:
    void lower_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
    void upper_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);

    void state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    void publish_robot_frames();

    void initialize_imus();

    void update_foot_frames();

    void visualize_joints();

    rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr m_state_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_upper_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_lower_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_state_subscriber;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_stance_foot_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_right_foot_on_ground_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_left_foot_on_ground_publisher;
    rclcpp::Publisher<march_shared_msgs::msg::CenterOfMass>::SharedPtr m_com_pos_publisher;
    rclcpp::Publisher<march_shared_msgs::msg::FeetHeightStamped>::SharedPtr m_feet_height_publisher;
    rclcpp::Publisher<march_shared_msgs::msg::Feet>::SharedPtr m_foot_impact_publisher;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_rviz_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_joint_broadcaster;
    JointEstimator m_joint_estimator;
    ImuEstimator m_imu_estimator;
    ExoEstimator m_exo_estimator;

    int m_current_stance_foot;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_joint_listener;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif