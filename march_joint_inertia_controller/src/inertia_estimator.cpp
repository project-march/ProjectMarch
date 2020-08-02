// Copyright 2020 Project March.
#include <cmath>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include "march_joint_inertia_controller/inertia_estimator.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

std::vector<double> absolute(const std::vector<double>& a)
{
  std::vector<double> b;
  b.resize(a.size());
  for (size_t i = 0; i < a.size(); ++i)
  {
    b[i] = abs(a[i]);
  }
  return b;
}

double mean(const std::vector<double>& a)
{
  double sum = 0.0;
  for (const double x : a)
  {
    sum += x;
  }
  if (a.size() == 0.0)
  {
    return 0.0;
  }
  return sum / a.size();
}

InertiaEstimator::InertiaEstimator(double lambda, size_t acc_size)
{
  lambda_ = lambda;
  acc_size_ = acc_size;
  joint_inertia_ = 0.0;

  // Size the buffers
  z1a.resize(sos_.size(), 0.0);
  z2a.resize(sos_.size(), 0.0);

  z1t.resize(sos_.size(), 0.0);
  z2t.resize(sos_.size(), 0.0);

  velocity_array_.resize(vel_size_, 0.0);

  acceleration_array_.resize(acc_size_, 0.0);

  filtered_acceleration_array_.resize(acc_size_, 0.0);

  joint_torque_.resize(torque_size_, 0.0);

  filtered_joint_torque_.resize(fil_tor_size_, 0.0);
}

double InertiaEstimator::getAcceleration(unsigned int index)
{
  return acceleration_array_[index];
}

void InertiaEstimator::setLambda(double lambda)
{
  lambda_ = lambda;
}
void InertiaEstimator::setAcc_size(size_t acc_size)
{
  acc_size_ = acc_size;
}

void InertiaEstimator::setNodeHandle(ros::NodeHandle& nh)
{
  nh_ = nh;
}

void InertiaEstimator::configurePublisher(const std::string& name)
{
  std::string publisher_name = "/inertia_publisher/" + name;
  pub_ = nh_.advertise<std_msgs::Float64>(publisher_name, 100);
}

void InertiaEstimator::publishInertia()
{
  std_msgs::Float64 msg;
  msg.data = joint_inertia_;
  pub_.publish(msg);
}

// Fills the buffers so that non-zero values may be computed by the inertia estimator
void InertiaEstimator::fill_buffers(double velocity, double effort, const ros::Duration& period)
{
  auto it = velocity_array_.begin();
  it = velocity_array_.begin();
  it = velocity_array_.insert(it, velocity);
  velocity_array_.resize(vel_size_);

  // Automatically fills the zero'th position of the acceleration array
  double acc = discrete_speed_derivative(period);
  it = acceleration_array_.begin();
  it = acceleration_array_.insert(it, acc);
  acceleration_array_.resize(acc_size_);

  it = joint_torque_.begin();
  it = joint_torque_.insert(it, effort);
  joint_torque_.resize(torque_size_);
}

void InertiaEstimator::apply_butter()
{
  // Dingen doen met de sos filter ofzo
  auto it = filtered_acceleration_array_.begin();
  double x = 1.0;

  int i = 0;
  for (const auto& so : sos_)
  {
    x = acceleration_array_[0];
    filtered_acceleration_array_[0] = so[0] * acceleration_array_[0] + z1a[i];
    z1a[i] = so[1] * x - so[4] * filtered_acceleration_array_[0] + z2a[i];
    z2a[i] = so[2] * x - so[5] * filtered_acceleration_array_[0];
    i++;
  }

  it = filtered_acceleration_array_.begin();
  it = filtered_acceleration_array_.insert(it, x);
  filtered_acceleration_array_.resize(acc_size_);

  x = 1.0;
  i = 0;
  for (const auto& so : sos_)
  {
    x = joint_torque_[0];
    filtered_joint_torque_[0] = so[0] * joint_torque_[0] + z1t[i];
    z1t[i] = so[1] * x - so[4] * filtered_joint_torque_[0] + z2t[i];
    z2t[i] = so[2] * x - so[5] * filtered_joint_torque_[0];
    i++;
  }
  it = filtered_joint_torque_.begin();
  it = filtered_joint_torque_.insert(it, x);
  filtered_joint_torque_.resize(fil_tor_size_);
}

// Estimate the inertia using the acceleration and torque
void InertiaEstimator::inertia_estimate()
{
  apply_butter();

  correlation_calculation();
  K_i_ = gain_calculation();
  K_a_ = alpha_calculation();
  const double torque_e = filtered_joint_torque_[0] - filtered_joint_torque_[1];
  const double acc_e = filtered_acceleration_array_[0] - filtered_acceleration_array_[1];
  joint_inertia_ = (torque_e - (acc_e * joint_inertia_)) * K_i_ * K_a_ + joint_inertia_;
}

// Calculate the alpha coefficient for the inertia estimate
double InertiaEstimator::alpha_calculation()
{
  double vib = std::max(std::min(vibration_calculation(), min_alpha_), max_alpha_);
  return (vib - min_alpha_) / (max_alpha_ - min_alpha_);
}

// Calculate the inertia gain for the inertia estimate
double InertiaEstimator::gain_calculation()
{
  return (corr_coeff_ * (filtered_acceleration_array_[0] - filtered_acceleration_array_[1])) /
         (lambda_ + corr_coeff_ * pow(filtered_acceleration_array_[0] - filtered_acceleration_array_[1], 2));
}

// Calculate the correlation coefficient of the acceleration buffer
void InertiaEstimator::correlation_calculation()
{
  corr_coeff_ =
      corr_coeff_ / (lambda_ + corr_coeff_ * pow(filtered_acceleration_array_[0] - filtered_acceleration_array_[1], 2));
  const double large_number = 10e8;
  if (corr_coeff_ > large_number)
  {
    corr_coeff_ = large_number;
  }
}

// Calculate the vibration based on the acceleration
double InertiaEstimator::vibration_calculation()
{
  mean_of_absolute_ = mean(absolute(filtered_acceleration_array_));
  absolute_of_mean_ = abs(mean(filtered_acceleration_array_));
  // Divide by zero protection, necessary when there has not been any acceleration yet
  if (absolute_of_mean_ == 0.0)
  {
    return 0.0;
  }
  return mean_of_absolute_ / absolute_of_mean_;
}

// Calculate a discrete derivative of the speed measurements
double InertiaEstimator::discrete_speed_derivative(const ros::Duration& period)
{
  return (velocity_array_[0] - velocity_array_[1]) / period.toSec();
}

void InertiaEstimator::init_p(unsigned int samples)
{
  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  double mean_value = mean(standard_deviation);
  double sum = 0;
  for (const auto& deviation : standard_deviation)
  {
    sum += std::pow(deviation - mean_value, 2);
  }
  corr_coeff_ = 100 * (sum / samples);
  return;
}
