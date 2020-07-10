// Copyright 2020 Project March.
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include "march_joint_inertia_controller/inertia_estimator.h"
#include "math.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

void absolute(std::vector<double> a, std::vector<double>& b)
{
  for (size_t i = 0; i < a.size(); ++i)
  {
    b[i] = abs(a[i]);
  }
}

int maximum(int a, int b, int c)
{
  int max = (a < b) ? b : a;
  return ((max < c) ? c : max);
}

double median(double a[], size_t sz)
{
  int uneven = sz % 2;
  // I do not want the original array to remain sorted because I may need it to compute a derivative or filter it.
  double b[sz];
  std::memcpy(b, a, sz);
  std::sort(b, b + sz);

  if (uneven == 1)
  {
    return b[sz / 2];
  }
  else if (uneven == 0)
  {
    return (b[sz / 2] + b[sz / 2 + 1]) / 2;
  }
  return 0.0;
}

double mean(std::vector<double> a)
{
  double sum = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
  {
    sum += a[i];
  }
  if (sum == 0.0)
  {
    return 0.0;
  }
  return sum / a.size();
}

InertiaEstimator::InertiaEstimator()
{
  // Size the buffers
  velocity_array_.resize(vel_size_);

  acceleration_array_.resize(acc_size_);

  filtered_acceleration_array_.resize(fil_acc_size_);

  joint_torque_.resize(torque_size_);

  filtered_joint_torque_.resize(fil_tor_size_);

  for (unsigned int i = 0; i < vel_size_; ++i)
  {
    velocity_array_[i] = 0.0;
  }

  for (unsigned int i = 0; i < acc_size_; ++i)
  {
    acceleration_array_[i] = 0.0;
  }

  for (unsigned int i = 0; i < torque_size_; ++i)
  {
    joint_torque_[i] = 0.0;
  }

  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  corr_coeff_ = 3000.0;

  // Setup Butterworth filter
}

InertiaEstimator::InertiaEstimator(hardware_interface::JointHandle joint)
{
  // Size the buffers
  velocity_array_.resize(vel_size_);

  acceleration_array_.resize(acc_size_);

  filtered_acceleration_array_.resize(fil_acc_size_);

  joint_torque_.resize(torque_size_);

  filtered_joint_torque_.resize(fil_tor_size_);

  setJoint(joint);

  for (unsigned int i = 0; i < vel_size_; ++i)
  {
    velocity_array_[i] = 0.0;
  }

  for (unsigned int i = 0; i < acc_size_; ++i)
  {
    acceleration_array_[i] = 0.0;
  }

  for (unsigned int i = 0; i < torque_size_; ++i)
  {
    joint_torque_[i] = 0.0;
  }

  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  corr_coeff_ = 3000.0;

  // Setup Butterworth filter
}

InertiaEstimator::~InertiaEstimator()
{
}

void InertiaEstimator::setJoint(hardware_interface::JointHandle joint)
{
  joint_ = joint;
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
bool InertiaEstimator::fill_buffers(const ros::Duration& period)
{
  auto it = velocity_array_.begin();

  it = velocity_array_.begin();
  it = velocity_array_.insert(it, joint_.getVelocity());
  velocity_array_.resize(vel_size_);

  // Automatically fills the zero'th position of the acceleration array
  discrete_speed_derivative(joint_.getVelocity(), period);
  acceleration_array_.resize(acc_size_);

  it = joint_torque_.begin();
  it = joint_torque_.insert(it, joint_.getEffort());
  joint_torque_.resize(torque_size_);
  return false;
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

// Fills the buffers so that non-zero values may be computed by the inertia estimator
bool InertiaEstimator::fill_buffers(double velocity, double effort)
{
  auto it = velocity_array_.begin();

  it = velocity_array_.begin();
  it = velocity_array_.insert(it, velocity);
  velocity_array_.resize(vel_size_);

  // Automatically fills the zero'th position of the acceleration array
  ros::Duration period(0.004);
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
  filtered_acceleration_array_.resize(fil_acc_size_);

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
  std::vector<double> b;
  b.resize(acc_size_);
  absolute(filtered_acceleration_array_, b);
  // moa = mean of the absolute
  moa_ = mean(b);
  // aom = absolute of the mean
  aom_ = abs(mean(filtered_acceleration_array_));
  if (aom_ == 0.0)
  {
    ROS_INFO("omaigod we gaan toch niet delen door nul hey");
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