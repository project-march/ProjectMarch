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
  for (size_t i = 0; i < a.size(); ++i)
  {
    sum += a[i];
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
  velocity_array_.resize(vel_size_);

  acceleration_array_.resize(acc_size_);

  filtered_acceleration_array_.resize(acc_size_);

  joint_torque_.resize(torque_size_);

  filtered_joint_torque_.resize(fil_tor_size_);
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

void InertiaEstimator::configurePublisher(std::string name)
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
bool InertiaEstimator::fill_buffers(double velocity, double effort, const ros::Duration& period)
{
  auto it = velocity_array_.begin();

  it = velocity_array_.begin();
  it = velocity_array_.insert(it, velocity);
  velocity_array_.resize(vel_size_);

  // Automatically fills the zero'th position of the acceleration array
  discrete_speed_derivative(velocity, period);
  acceleration_array_.resize(acc_size_);

  it = joint_torque_.begin();
  it = joint_torque_.insert(it, effort);
  joint_torque_.resize(torque_size_);
  return false;
}

void InertiaEstimator::apply_butter()
{
  // Dingen doen met de sos filter ofzo
  // Also doe pushback dingen anders dan werkt dit niet
  std::vector<double> z = { 0.0, 0.0, 0.0 };
  auto it = filtered_acceleration_array_.begin();
  auto x_n = acceleration_array_[0];
  double x;

  for (unsigned int i = 0; i < (sizeof(sos_) / sizeof(sos_[0])); ++i)
  {
    x_n = acceleration_array_[0];
    x = sos_[i][0] * x_n + z[0];
    z[0] = sos_[i][1] * x_n - sos_[i][3] * x + z[1];
    z[1] = sos_[i][2] * x_n - sos_[i][4] * x;

    //    temp *= (acceleration_array_[0] * sos_[i][0] + acceleration_array_[1] * sos_[i][1] +
    //             acceleration_array_[2] * sos_[i][2]) /
    //            (acceleration_array_[0] * sos_[i][3] + acceleration_array_[1] * sos_[i][4] +
    //             acceleration_array_[2] * sos_[i][5]);
  }
  it = filtered_acceleration_array_.begin();
  it = filtered_acceleration_array_.insert(it, x);
  filtered_acceleration_array_.resize(acc_size_);

  z = { 0.0, 0.0, 0.0 };
  for (unsigned int i = 0; i < (sizeof(sos_) / sizeof(sos_[0])); ++i)
  {
    x_n = joint_torque_[0];
    x = sos_[i][0] * x_n + z[0];
    z[0] = sos_[i][1] * x_n - sos_[i][3] * x + z[1];
    z[1] = sos_[i][2] * x_n - sos_[i][4] * x;
    //    temp *= (joint_torque_[0] * sos_[i][0] + joint_torque_[1] * sos_[i][1] + joint_torque_[2] * sos_[i][2]) /
    //            (joint_torque_[0] * sos_[i][3] + joint_torque_[1] * sos_[i][4] + joint_torque_[2] * sos_[i][5]);
  }
  it = filtered_joint_torque_.begin();
  it = filtered_joint_torque_.insert(it, x);
  filtered_joint_torque_.resize(fil_tor_size_);

  // temp = 1.0;
}

// Estimate the inertia using the acceleration and torque
void InertiaEstimator::inertia_estimate()
{
  double torque_e;
  double acc_e;
  apply_butter();

  correlation_calculation();
  K_i_ = gain_calculation();
  K_a_ = alpha_calculation();
  torque_e = filtered_joint_torque_[0] - filtered_joint_torque_[1];
  acc_e = filtered_acceleration_array_[0] - filtered_acceleration_array_[1];
  joint_inertia_ = (torque_e - (acc_e * joint_inertia_)) * K_i_ * K_a_ + joint_inertia_;
}

// Calculate the alpha coefficient for the inertia estimate
double InertiaEstimator::alpha_calculation()
{
  double vib = vibration_calculation();
  if (vib < min_alpha_)
  {
    vib = min_alpha_;
  }
  else if (vib > max_alpha_)
  {
    vib = max_alpha_;
  }
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
  double large_number = pow(10, 8);
  if (corr_coeff_ > large_number)
  {
    corr_coeff_ = large_number;
  }
}

// Calculate the vibration based on the acceleration
double InertiaEstimator::vibration_calculation()
{
  // moa = mean of the absolute
  moa_ = mean(absolute(filtered_acceleration_array_));
  // aom = absolute of the mean
  aom_ = abs(mean(filtered_acceleration_array_));
  // Divide by zero protection, necessary when exo hasn't homed yet
  if (aom_ == 0.0)
  {
    return 0.0;
  }
  return moa_ / aom_;
}

// Calculate a discrete derivative of the speed measurements
void InertiaEstimator::discrete_speed_derivative(double velocity, const ros::Duration& period)
{
  auto it = acceleration_array_.begin();
  it = acceleration_array_.insert(it, (velocity - velocity_array_[1]) / period.toSec());
}

void InertiaEstimator::init_p(unsigned int samples)
{
  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  double mean_value = mean(standard_deviation);
  double sum = 0;
  for (unsigned int i = 0; i < standard_deviation.size(); ++i)
  {
    sum += pow((standard_deviation[i] - mean_value), 2);
  }
  corr_coeff_ = 100 * (sum / samples);
  return;
}