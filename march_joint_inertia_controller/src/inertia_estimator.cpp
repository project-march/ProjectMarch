// Copyright 2020 Project March.
#include "march_joint_inertia_controller/inertia_estimator.h"
#include "ros/ros.h"

void absolute(std::vector<double> a, std::vector<double> b)
{
  for (size_t i = 0; i < a.size(); ++i)
  {
    if (a[i] < 0)
    {
      b[i] = a[i] - 1;
    }
  }
}

// Compute the absolute value of a double and return it.
double absolute(double a)
{
  if (a < 0)
  {
    return a * (-1);
  }
  else
  {
    return a;
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

  ros::Duration first(0.004);
  // Bad practice (de time attribute), maar wat anders? en gaat dit wel goed met elke keer dat we data ontvangen?
  // Krijgen we niet dubbele waarden op deze manier?
  fill_buffers(first);

  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  corr_coeff_ = 0.0;

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

  ros::Duration first(0.004);
  // Bad practice (de time attribute), maar wat anders? en gaat dit wel goed met elke keer dat we data ontvangen?
  // Krijgen we niet dubbele waarden op deze manier?
  fill_buffers(first);

  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  corr_coeff_ = 0.0;

  // Setup Butterworth filter
}

InertiaEstimator::~InertiaEstimator()
{
}

void InertiaEstimator::setJoint(hardware_interface::JointHandle joint)
{
  joint_ = joint;
}

// Fills the buffers so that non-zero values may be computed by the inertia estimator
bool InertiaEstimator::fill_buffers(const ros::Duration& period)
{
  auto it = velocity_array_.begin();

  it = velocity_array_.begin();
  it = velocity_array_.insert(it, joint_.getVelocity());
  velocity_array_.resize(vel_size_);

  // Automatically fills the zero'th position of the acceleration array
  discrete_speed_derivative(period);
  acceleration_array_.resize(acc_size_);

  it = joint_torque_.begin();
  it = joint_torque_.insert(it, joint_.getEffort());
  joint_torque_.resize(torque_size_);
  return false;
}

void InertiaEstimator::apply_butter()
{
  // Dingen doen met de sos filter ofzo
  // Also doe pushback dingen anders dan werkt dit niet
  double temp = 1.0;
  auto it = filtered_acceleration_array_.begin();

  for (unsigned int i = 0; i < (sizeof(sos_) / sizeof(sos_[0])); ++i)
  {
    temp *= (acceleration_array_[0] * sos_[i][0] + acceleration_array_[1] * sos_[i][1] +
             acceleration_array_[2] * sos_[i][2]) /
            (acceleration_array_[0] * sos_[i][3] + acceleration_array_[1] * sos_[i][4] +
             acceleration_array_[2] * sos_[i][5]);
  }
  it = filtered_acceleration_array_.begin();
  it = filtered_acceleration_array_.insert(it, temp);
  filtered_acceleration_array_.resize(fil_acc_size_);

  temp = 1.0;
  for (unsigned int i = 0; i < (sizeof(sos_) / sizeof(sos_[0])); ++i)
  {
    temp *= (joint_torque_[0] * sos_[i][0] + joint_torque_[1] * sos_[i][1] + joint_torque_[2] * sos_[i][2]) /
            (joint_torque_[0] * sos_[i][3] + joint_torque_[1] * sos_[i][4] + joint_torque_[2] * sos_[i][5]);
  }
  it = filtered_joint_torque_.begin();
  it = filtered_joint_torque_.insert(it, temp);
  filtered_joint_torque_.resize(fil_tor_size_);

  temp = 1.0;
}

// Estimate the inertia using the acceleration and torque
void InertiaEstimator::inertia_estimate()
{
  double K_i;
  double K_a;
  double torque_e;
  double acc_e;
  apply_butter();

  correlation_calculation();
  K_i = gain_calculation();
  K_a = alpha_calculation();
  torque_e = filtered_joint_torque_[0] - filtered_joint_torque_[1];
  acc_e = filtered_acceleration_array_[0] - filtered_acceleration_array_[1];
  joint_inertia_ = (torque_e - (acc_e * joint_inertia_)) * K_i * K_a + joint_inertia_;
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
  absolute(filtered_acceleration_array_, b);
  // moa = mean of the absolute
  double moa = mean(b);
  // aom = absolute of the mean
  double aom = absolute(mean(filtered_acceleration_array_));
  return moa / aom;
}

// Calculate a discrete derivative of the speed measurements
void InertiaEstimator::discrete_speed_derivative(const ros::Duration& period)
{
  auto it = acceleration_array_.begin();
  it = acceleration_array_.insert(it, (joint_.getVelocity() - velocity_array_[1]) / period.toSec());
}