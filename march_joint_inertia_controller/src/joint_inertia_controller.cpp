// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_inertia_controller.h"
#include <math.h>

using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

// Compute the absolute value of array a and store it back in b so that a remains intact.
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

  if (num_joints_ == 0)
  {
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    try
    {
      joints_.push_back(hw->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }

  // Fill the buffers
  velocity_array_.resize(num_joints_);
  for (unsigned int j = 0; j < num_joints_; ++j)
    velocity_array_[j].resize(vel_size_);

  acceleration_array_.resize(num_joints_);
  for (unsigned int j = 0; j < num_joints_; ++j)
    acceleration_array_[j].resize(acc_size_);

  filtered_acceleration_array_.resize(num_joints_);
  for (unsigned int j = 0; j < num_joints_; ++j)
    filtered_acceleration_array_[j].resize(fil_acc_size_);

  joint_torque_.resize(num_joints_);
  for (unsigned int j = 0; j < num_joints_; ++j)
    joint_torque_[j].resize(torque_size_);

  filtered_joint_torque_.resize(num_joints_);
  for (unsigned int j = 0; j < num_joints_; ++j)
    filtered_joint_torque_[j].resize(fil_tor_size_);

  corr_coeff_.resize(num_joints_);
  joint_inertia_.resize(num_joints_);

  int longest_array = maximum(velocity_array_[0].size(), acceleration_array_[0].size(), joint_torque_[0].size());

  ros::Duration first(0.004);
  for (int i = 0; i < longest_array; i++)
  {
    // Bad practice (de time attribute), maar wat anders? en gaat dit wel goed met elke keer dat we data ontvangen?
    // Krijgen we niet dubbele waarden op deze manier?
    this->fill_buffers(first);
  }

  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  for (unsigned int j = 0; j < num_joints_; ++j)
    corr_coeff_[j] = 0.0;

  // Setup Butterworth filter

  commands_buffer_.writeFromNonRT(std::vector<double>(num_joints_, 0.0));

  sub_command_ = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &InertiaController::commandCB, this);

  return true;
}

// Wait what is difference between starting and init????
void InertiaController::starting(const ros::Time& /* time */)
{
  // Add ros delay or something so that we fill these buffers with actual values.
  for (unsigned int j = 0; j < num_joints_; ++j)
  {
    // init_pos_ = this->march_robot_->getJoint(j).getPosition();
    for (size_t i = 1; i < velocity_array_[j].size(); ++i)
    {
      velocity_array_[j][i] = 0.0;
    }

    for (size_t i = 1; i < acceleration_array_[j].size(); ++i)
    {
      acceleration_array_[j][i] = 0.0;
    }

    for (size_t i = 1; i < joint_torque_[j].size(); ++i)
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
  std::vector<double>& commands = *commands_buffer_.readFromRT();
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    joints_[i].setCommand(commands[i]);
  }
}
void InertiaController::stopping(const ros::Time& /* time */)
{
}

void InertiaController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  if (msg->data.size() != num_joints_)
  {
    ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints ("
                                              << num_joints_ << ")! Not executing!");
    return;
  }
  commands_buffer_.writeFromNonRT(msg->data);
}

// Fills the buffers so that non-zero values may be computed by the inertia estimator
bool InertiaController::fill_buffers(const ros::Duration& period)
{
  auto it = velocity_array_[0].begin();
  for (unsigned int j = 0; j < num_joints_; ++j)
  {
    it = velocity_array_[j].begin();
    it = velocity_array_[j].insert(it, joints_[j].getVelocity());
    velocity_array_.resize(vel_size_);

    // Automatically fills the zero'th position of the acceleration array
    this->discrete_speed_derivative(j, period);
    acceleration_array_[j].resize(acc_size_);

    it = joint_torque_[j].begin();
    it = joint_torque_[j].insert(it, joints_[j].getEffort());
    joint_torque_[j].resize(torque_size_);
  }
  return false;
}

void InertiaController::apply_butter()
{
  // Dingen doen met de sos filter ofzo
  // Also doe pushback dingen anders dan werkt dit niet
  double temp = 1.0;
  auto it = filtered_acceleration_array_[0].begin();
  for (unsigned int j = 0; j < num_joints_; ++j)
  {
    for (unsigned int i = 0; i < (sizeof(sos_) / sizeof(sos_[0])); ++i)
    {
      temp *= (acceleration_array_[j][0] * sos_[i][0] + acceleration_array_[j][1] * sos_[i][1] +
               acceleration_array_[j][2] * sos_[i][2]) /
              (acceleration_array_[j][0] * sos_[i][3] + acceleration_array_[j][1] * sos_[i][4] +
               acceleration_array_[j][2] * sos_[i][5]);
    }
    it = filtered_acceleration_array_[j].begin();
    it = filtered_acceleration_array_[j].insert(it, temp);
    filtered_acceleration_array_[j].resize(fil_acc_size_);

    temp = 1.0;
    for (unsigned int i = 0; i < (sizeof(sos_) / sizeof(sos_[0])); ++i)
    {
      temp *= (joint_torque_[j][0] * sos_[i][0] + joint_torque_[j][1] * sos_[i][1] + joint_torque_[j][2] * sos_[i][2]) /
              (joint_torque_[j][0] * sos_[i][3] + joint_torque_[j][1] * sos_[i][4] + joint_torque_[j][2] * sos_[i][5]);
    }
    it = filtered_joint_torque_[j].begin();
    it = filtered_joint_torque_[j].insert(it, temp);
    filtered_joint_torque_[j].resize(fil_tor_size_);

    temp = 1.0;
  }
}

// Estimate the inertia using the acceleration and torque
void InertiaController::inertia_estimate()
{
  double K_i[num_joints_];
  double K_a[num_joints_];
  double torque_e[num_joints_];
  double acc_e[num_joints_];
  this->apply_butter();
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    this->correlation_calculation(i);
    K_i[i] = this->gain_calculation(i);
    K_a[i] = this->alpha_calculation(i);
    torque_e[i] = filtered_joint_torque_[i][0] - filtered_joint_torque_[i][1];
    acc_e[i] = filtered_acceleration_array_[i][0] - filtered_acceleration_array_[i][1];
    joint_inertia_[i] = (torque_e[i] - (acc_e[i] * joint_inertia_[i])) * K_i[i] * K_a[i] + joint_inertia_[i];
  }
}

// Calculate the alpha coefficient for the inertia estimate
double InertiaController::alpha_calculation(int joint_nr)
{
  double vib = this->vibration_calculation(joint_nr);
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
double InertiaController::gain_calculation(int jn)
{
  return (corr_coeff_[jn] * (filtered_acceleration_array_[jn][0] - filtered_acceleration_array_[jn][1])) /
         (lambda_ +
          corr_coeff_[jn] * pow(filtered_acceleration_array_[jn][0] - filtered_acceleration_array_[jn][1], 2));
}

// Calculate the correlation coefficient of the acceleration buffer
void InertiaController::correlation_calculation(int jn)
{
  corr_coeff_[jn] =
      corr_coeff_[jn] /
      (lambda_ + corr_coeff_[jn] * pow(filtered_acceleration_array_[jn][0] - filtered_acceleration_array_[jn][1], 2));
  double large_number = pow(10, 8);
  if (corr_coeff_[jn] > large_number)
  {
    corr_coeff_[jn] = large_number;
  }
}

// Calculate the vibration based on the acceleration
double InertiaController::vibration_calculation(int joint_nr)
{
  std::vector<double> b;
  absolute(filtered_acceleration_array_[joint_nr], b);
  // moa = mean of the absolute
  double moa = mean(b);
  // aom = absolute of the mean
  double aom = absolute(mean(filtered_acceleration_array_[joint_nr]));
  return moa / aom;
}

// Calculate a discrete derivative of the speed measurements
void InertiaController::discrete_speed_derivative(int joint_nr, const ros::Duration& period)
{
  auto it = acceleration_array_[joint_nr].begin();
  it = acceleration_array_[joint_nr].insert(it, (joints_[joint_nr].getVelocity() - velocity_array_[joint_nr][1]) /
                                                    period.toSec());
}

}  // namespace joint_inertia_controller
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller::InertiaController, controller_interface::ControllerBase);
