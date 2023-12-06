#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "imu_estimator.hpp"
#include "joint_estimator.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "march_shared_msgs/msg/feet.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/robot_state.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/srv/get_current_stance_leg.hpp"
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
#include "march_shared_msgs/srv/get_task_report.hpp"
#include "march_shared_msgs/srv/get_current_joint_positions.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

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

    void publishFootPositions(); 

    void initialize_imus();

    void update_foot_frames();

    void visualize_joints();

    void handleTaskReportRequest(const std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Response> response);
    void handleJointPositionsRequest(const std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response);
        
    void setStanceFoot();
    void stanceFootServiceCallback(const std::shared_ptr<march_shared_msgs::srv::GetCurrentStanceLeg::Request>,
        std::shared_ptr<march_shared_msgs::srv::GetCurrentStanceLeg::Response> response);

    rclcpp::Publisher<march_shared_msgs::msg::RobotState>::SharedPtr m_state_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_upper_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_lower_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_state_subscriber;

    rclcpp::Publisher<march_shared_msgs::msg::FeetHeightStamped>::SharedPtr m_feet_height_publisher;

    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_feet_position_publisher; 

    rclcpp::Service<march_shared_msgs::srv::GetCurrentStanceLeg>::SharedPtr m_current_stance_foot_service;


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_rviz_publisher;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;

    rclcpp::Service<march_shared_msgs::srv::GetTaskReport>::SharedPtr task_report_service_;
    rclcpp::Service<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr joint_positions_service_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_joint_broadcaster;
    JointEstimator m_joint_estimator;
    ImuEstimator m_imu_estimator;
    ExoEstimator m_exo_estimator;

    int m_current_stance_foot;
    std::vector<double> m_current_joint_positions;
    std::vector<double> m_current_joint_velocities;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_joint_listener;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif