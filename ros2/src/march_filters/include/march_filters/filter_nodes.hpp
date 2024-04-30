/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_FILTERS__FILTER_NODES_HPP_
#define MARCH_FILTERS__FILTER_NODES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "march_filters/my_mean_filter.hpp"
#include "march_filters/my_median_filter.hpp"
#include "march_filters/my_transfer_function.hpp"

#include <vector>

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define AXIS_W 3

#define NUM_CHANNELS_IMU_ACC 3
#define NUM_CHANNELS_IMU_GYRO 3
#define NUM_CHANNELS_TORQUE 10

class FiltersNode : public rclcpp::Node
{
public:
  FiltersNode();
  ~FiltersNode() = default;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  std::vector<MyMeanFilter::SharedPtr> m_imu_acc_mean_filters;
  std::vector<MyMeanFilter::SharedPtr> m_imu_gyro_mean_filters;
  std::vector<MyMeanFilter::SharedPtr> m_torque_mean_filters;

  const unsigned int m_imu_acc_window_size = 125;
  const unsigned int m_imu_gyro_window_size = 125;
  const unsigned int m_torque_window_size = 125;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;
};

#endif  // MARCH_FILTERS__FILTER_NODES_HPP_