// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H
#define MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <pluginlib/class_list_macros.h>

using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace joint_inertia_controller
{
class InertiaController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

  // Applies the Butterworth filter over the last two samples and returns the resulting filtered value.
  void apply_butter();

  // Estimate the inertia using the acceleration and torque
  void inertia_estimate();
  // Calculate a discrete derivative of the speed measurements
  void discrete_speed_derivative(const ros::Duration&);
  // Calculate the alpha coefficient for the inertia estimate
  double alpha_calculation();
  // Calculate the inertia gain for the inertia estimate
  double gain_calculation();
  // Calculate the correlation coefficient of the acceleration buffer
  void correlation_calculation();
  // Calculate the vibration based on the acceleration
  double vibration_calculation();

  // Fill the buffers with corresponding values. Are arrays efficient enough/is this method efficient enough? There
  // exist vector methods such as pushback() and resize as well. HALP!
  bool fill_buffers(const ros::Duration& period);

private:
  hardware_interface::JointHandle joint_;
  double init_pos_;
  std::string joint_names_;
  int num_joints_;

  float min_alpha_ = 0.4;  // You might want to be able to adjust this value from a yaml/launch file
  float max_alpha_ = 0.9;  // You might want to be able to adjust this value from a yaml/launch file

  double sos_[3][6] = {
    { 2.31330497e-05, 4.62660994e-05, 2.31330497e-05, 1.00000000e+00, -1.37177561e+00, 4.75382129e-01 },
    { 1.00000000e+00, 2.00000000e+00, 1.00000000e+00, 1.00000000e+00, -1.47548044e+00, 5.86919508e-01 },
    { 1.00000000e+00, 2.00000000e+00, 1.00000000e+00, 1.00000000e+00, -1.69779140e+00, 8.26021017e-01 }
  };
  // namespace joint_inertia_controller

  // Replace the 8 with he number of joints later
  // Of length 12 for the acceleration buffer
  double velocity_array_[march->num_joints_][12];
  // Of length 12 for the alpha calculation
  double acceleration_array_[num_joints_][12];
  // Of length 2 for the error calculation
  double filtered_acceleration_array_[num_joints_][2];
  // Of length 3 for the butterworth filter
  double joint_torque_[num_joints_][3];
  // Of length 2 for the error calculation
  double filtered_joint_torque_[num_joints_][2];

  // Correlation coefficient used to calculate the inertia gain
  double corr_coeff_[num_joints_];
  double joint_inertia_[num_joints_];
  double lambda_ = 0.96;  // You might want to be able to adjust this value from a yaml/launch file
};
}  // namespace joint_inertia_controller

#endif  // MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H
