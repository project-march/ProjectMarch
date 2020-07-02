// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_inertia_controller.h"
#include <math.h>

using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

// Compute the absolute value of array a and store it back in b so that a remains intact.
void absolute(double a[], double b[], size_t sz)
{
  for (size_t i = 0; i < sz; ++i)
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

double mean(double a[], size_t sz)
{
  double sum = 0.0;
  for (size_t i = 0; i < sz; ++i)
  {
    sum += a[i];
  }
  return sum / sz;
}

namespace joint_inertia_controller
{
bool InertiaController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  if (!nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("No joint_names specified");
    return false;
  }
  num_joints_ = joint_names_.size();
  joint_ = hw->getHandle(joint_names_);

  // Fill the buffers
  int vel_len = sizeof(velocity_array_[0]) / sizeof(velocity_array_[0][0]);
  int acc_len = sizeof(acceleration_array_[0]) / sizeof(acceleration_array_[0][0]);
  int tor_len = sizeof(joint_torque_[0]) / sizeof(joint_torque_[0][0]);

  int longest_array = maximum(vel_len, acc_len, tor_len);

  ros::Duration first(0.004);
  for (int i = 0; i < longest_array; i++)
  {
    // Bad practice (de time attribute), maar wat anders? en gaat dit wel goed met elke keer dat we data ontvangen?
    // Krijgen we niet dubbele waarden op deze manier?
    this->fill_buffers(first);
  }

  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  corr_coeff_ = 0.0;

  // Setup Butterworth filter

  return true;
}

// Wait what is difference between starting and init????
void InertiaController::starting(const ros::Time& /* time */)
{
  // Add ros delay or something so that we fill these buffers with actual values.
  for (int j = 0; j < num_joints_; ++j)
  {
    init_pos_ = this->march_robot_->getJoint(j).getPosition();
    for (size_t i = 1; i < sizeof(velocity_array_[j]) / sizeof(velocity_array_[j][0]); ++i)
    {
      velocity_array_[j][i] = 0.0;
    }

    for (size_t i = 1; i < sizeof(acceleration_array_[j]) / sizeof(acceleration_array_[j][0]); ++i)
    {
      acceleration_array_[j][i] = 0.0;
    }

    for (size_t i = 1; i < sizeof(joint_torque_[j]) / sizeof(joint_torque_[j][0]); ++i)
    {
      joint_torque_[j][i] = 0.0;
    }
  }
}

void InertiaController::update(const ros::Time& /* time */, const ros::Duration& period /* period */)
{
  this->fill_buffers(period);
  this->inertia_estimate();
  // TO DO: Provide lookup table for gain selection
  // TO DO: apply PID control
  joint_.setCommand(0);
}
void InertiaController::stopping(const ros::Time& /* time */)
{
}

// Fills the buffers so that non-zero values may be computed by the inertia estimator
bool InertiaController::fill_buffers(const ros::Duration& period)
{
  for (int j = 0; j < num_joints_; ++j)
  {
    for (size_t i = 1; i < sizeof(velocity_array_[j]) / sizeof(velocity_array_[j][0]); ++i)
    {
      velocity_array_[j][i] = velocity_array_[j][i - 1];
    }
    velocity_array_[j][0] = joint_velocity_[j];

    for (size_t i = 1; i < sizeof(acceleration_array_[j]) / sizeof(acceleration_array_[j][0]); ++i)
    {
      acceleration_array_[j][i] = acceleration_array_[j][i - 1];
    }
    // Automatically fills the zero'th position of the acceleration array
    this->discrete_speed_derivative(period);

    for (size_t i = 1; i < sizeof(joint_torque_[j]) / sizeof(joint_torque_[j][0]); ++i)
    {
      joint_torque_[j][i] = joint_torque_[j][i - 1];
    }
    joint_torque_[j][0] = joint_effort_[j];
  }
  return false;
}

void InertiaController::apply_butter()
{
  // Dingen doen met de sos filter ofzo
  // Also doe pushback dingen anders dan werkt dit niet
  double temp = 1.0;
  for (int j = 0; j < num_joints_; ++j)
  {
    for (int i = 0; i < 3; ++i)
    {
      temp *= (acceleration_array_[j][0] * sos_[i][0] + acceleration_array_[j][1] * sos_[i][1] +
               acceleration_array_[j][2] * sos_[i][2]) /
              (acceleration_array_[j][0] * sos_[i][3] + acceleration_array_[j][1] * sos_[i][4] +
               acceleration_array_[j][2] * sos_[i][5]);
    }
    // Push back the new value
    filtered_acceleration_array_[j][1] = filtered_acceleration_array_[j][0];
    filtered_acceleration_array_[j][0] = temp;

    temp = 1.0;
    for (int i = 0; i < 3; ++i)
    {
      temp *= (joint_torque_[j][0] * sos_[i][0] + joint_torque_[j][1] * sos_[i][1] + joint_torque_[j][2] * sos_[i][2]) /
              (joint_torque_[j][0] * sos_[i][3] + joint_torque_[j][1] * sos_[i][4] + joint_torque_[j][2] * sos_[i][5]);
    }
    // Push back the new value
    filtered_joint_torque_[j][1] = filtered_joint_torque_[j][0];
    filtered_joint_torque_[j][0] = temp;

    temp = 1.0;
  }
}

// Estimate the inertia using the acceleration and torque
void InertiaController::inertia_estimate()
{
  double K_i[num_joints];
  double K_a[num_joints];
  double torque_e[num_joints_];
  double acc_e[num_joints_];
  for (int i = 0; i < num_joints_; ++i)
  {
    this->apply_butter(i);
    this->correlation_calculation(i);
    K_i[i] = this->gain_calculation();
    K_a[i] = this->alpha_calculation();
    torque_e[i] = filtered_joint_torque_[j][0] - filtered_joint_torque_[j][1];
    acc_e[i] = filtered_acceleration_array_[j][0] - filtered_acceleration_array_[j][1];
    joint_inertia_[i] = (torque_e - (acc_e * joint_inertia_)) * K_i * K_a + joint_inertia_;
  }
}

// Calculate the alpha coefficient for the inertia estimate
double InertiaController::alpha_calculation()
{
  double vib = this->vibration_calculation();
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
double InertiaController::gain_calculation(int joint_nr)
{
  return (corr_coeff_[joint_nr] * (filtered_acceleration_array_[j][0] - filtered_acceleration_array_[j][1])) /
         (lambda_ +
          corr_coeff_[joint_nr] * pow(filtered_acceleration_array_[j][0] - filtered_acceleration_array_[j][1], 2));
}

// Calculate the correlation coefficient of the acceleration buffer
void InertiaController::correlation_calculation()
{
  for (int i = 0; i < num_joints_; ++i)
  {
    corr_coeff_[i] =
        corr_coeff_[i] /
        (lambda_ + corr_coeff_[i] * pow(filtered_acceleration_array_[j][0] - filtered_acceleration_array_[j][1], 2));
    double large_number = pow(10, 8);
    if (corr_coeff_[i] > large_number)
    {
      corr_coeff_[i] = large_number;
    }
  }
}

// Calculate the vibration based on the acceleration
double InertiaController::vibration_calculation()
{
  size_t len = sizeof(filtered_acceleration_array_[j]) / sizeof(filtered_acceleration_array_[j][0]);
  double b[len];
  absolute(filtered_acceleration_array_[j], b, len);
  // moa = mean of the absolute
  double moa = mean(b, len);
  // aom = absolute of the mean
  double aom = absolute(mean(filtered_acceleration_array_[j], len));
  return moa / aom;
}

// Calculate a discrete derivative of the speed measurements
void InertiaController::discrete_speed_derivative(const ros::Duration& period)
{
  for (int i = 0; i < num_joints_; ++i)
  {
    acceleration_array_[i][0] = (joint_velocity[i] - velocity_array_[i][1]) / period.toSec();
  }
}

}  // namespace joint_inertia_controller
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller::InertiaController, controller_interface::ControllerBase);
