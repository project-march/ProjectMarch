/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__FILTERS__FILTER_NODES_HPP_
#define MARCH_STATE_ESTIMATOR__FILTERS__FILTER_NODES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "march_state_estimator/filters/my_mean_filter.hpp"
#include "march_state_estimator/filters/my_median_filter.hpp"
#include "march_state_estimator/filters/my_transfer_function.hpp"

#include <vector>
#include <unordered_map>

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define AXIS_W 3

#define NUM_CHANNELS_IMU_ACC 3
#define NUM_CHANNELS_IMU_GYRO 3
#define NUM_CHANNELS_TORQUE 10

#define LOWER_LIMIT 0
#define UPPER_LIMIT 1
#define NUM_LIMITS 2

class FiltersNode : public rclcpp::Node
{
public:
  FiltersNode();
  ~FiltersNode() = default;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  inline bool jointNameBlacklisted(const std::string& joint_name) {
    return std::find(m_blacklist_joint_names.begin(), m_blacklist_joint_names.end(), joint_name) != m_blacklist_joint_names.end();
  }

  std::vector<MyMeanFilter::SharedPtr> m_imu_acc_mean_filters;
  std::vector<MyMeanFilter::SharedPtr> m_imu_gyro_mean_filters;
  std::vector<MyMeanFilter::SharedPtr> m_torque_mean_filters;

  std::unordered_map<std::string, double> m_joint_position_map;
  std::unordered_map<std::string, double> m_joint_velocity_map;
  std::unordered_map<std::string, double> m_joint_effort_map;
  std::unordered_map<std::string, double[NUM_LIMITS]> m_joint_position_limits_map;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;

  const unsigned int m_imu_acc_window_size = 125;
  const unsigned int m_imu_gyro_window_size = 125;
  const unsigned int m_torque_window_size = 125;

  std::vector<std::string> m_blacklist_joint_names;
};

#endif  // MARCH_STATE_ESTIMATOR__FILTERS__FILTER_NODES_HPP_