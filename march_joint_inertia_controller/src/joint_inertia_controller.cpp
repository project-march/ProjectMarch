// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_inertia_controller.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

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
InertiaController::InertiaController()
    : loop_count_(0)
{}

InertiaController::~InertiaController()
{
  sub_command_.shutdown();
}

bool InertiaController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  if (!nh.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint_names specified");
    return false;
  }

  // Size the buffers
  velocity_array_.resize(vel_size_);

  acceleration_array_.resize(acc_size_);

  filtered_acceleration_array_.resize(fil_acc_size_);

  joint_torque_.resize(torque_size_);

  filtered_joint_torque_.resize(fil_tor_size_);

  ros::Duration first(0.004);
  // Bad practice (de time attribute), maar wat anders? en gaat dit wel goed met elke keer dat we data ontvangen?
  // Krijgen we niet dubbele waarden op deze manier?
  this->fill_buffers(first);

  // Setup the initial value for the correlation coefficient 100*standarddeviation(acceleration)^2
  corr_coeff_ = 0.0;

  // Setup Butterworth filter

  sub_command_ = nh.subscribe<std_msgs::Float64>("command", 1, &InertiaController::commandCB, this);

  // Get joint handle from hardware interface
  joint_ = hw->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void InertiaController::setGains(const double& p, const double& i, const double& d, const double& i_max,
                                 const double& i_min, const bool& antiwindup)
{
  pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
}

void InertiaController::getGains(double& p, double& i, double& d, double& i_max, double& i_min, bool& antiwindup)
{
  pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
}

void InertiaController::getGains(double& p, double& i, double& d, double& i_max, double& i_min)
{
  bool dummy;
  pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
}

void InertiaController::printDebug()
{
  pid_controller_.printValues();
}

std::string InertiaController::getJointName()
{
  return joint_.getName();
}

double InertiaController::getPosition()
{
  return joint_.getPosition();
}

// Wait what is difference between starting and init????
void InertiaController::starting(const ros::Time& /* time */)
{
  double pos_command = joint_.getPosition();

  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false;

  command_.initRT(command_struct_);

  pid_controller_.reset();

  // init_pos_ = this->march_robot_->getJoint(j).getPosition();
  for (size_t i = 1; i < velocity_array_.size(); ++i)
  {
    velocity_array_[i] = 0.0;
  }

  for (size_t i = 1; i < acceleration_array_.size(); ++i)
  {
    acceleration_array_[i] = 0.0;
  }

  for (size_t i = 1; i < joint_torque_.size(); ++i)
  {
    joint_torque_[i] = 0.0;
  }
}

void InertiaController::update(const ros::Time& /* time */, const ros::Duration& period /* period */)
{
  command_struct_ = *(command_.readFromRT());
  double command_position = command_struct_.position_;
  double command_velocity = command_struct_.velocity_;
  bool has_velocity_ = command_struct_.has_velocity_;

  double error, vel_error;
  double commanded_effort;

  double current_position = joint_.getPosition();

  // Example says to chweck limits here, but we already do that in the validate() function in the HWI_node

  if (joint_urdf_->type == urdf::Joint::REVOLUTE)
  {
    angles::shortest_angular_distance_with_large_limits(current_position, command_position, joint_urdf_->limits->lower,
                                                        joint_urdf_->limits->upper, error);
  }
  else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(current_position, command_position);
  }
  else  // prismatic
  {
    error = command_position - current_position;
  }

  // Decide which of the two PID computeCommand() methods to call
  if (has_velocity_)
  {
    // Compute velocity error if a non-zero velocity command was given
    vel_error = command_velocity - joint_.getVelocity();

    // Set the PID error and compute the PID command with nonuniform
    // time step size. This also allows the user to pass in a precomputed derivative error.
    commanded_effort = pid_controller_.computeCommand(error, vel_error, period);
  }
  else
  {
    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    commanded_effort = pid_controller_.computeCommand(error, period);
  }

//  this->fill_buffers(period);
//  this->inertia_estimate();
  // TO DO: Provide lookup table for gain selection
  // TO DO: apply PID control

  joint_.setCommand(commanded_effort);

  // Example says to update state p[ublisher but we already do that somewhere else?

  loop_count_++;
}
void InertiaController::stopping(const ros::Time& /* time */)
{
}

void InertiaController::commandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

void InertiaController::setCommand(double pos_command, double vel_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.velocity_ = vel_command;
  command_struct_.has_velocity_ = true;

  command_.writeFromNonRT(command_struct_);
}

void InertiaController::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ =
      false;  // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}

// Fills the buffers so that non-zero values may be computed by the inertia estimator
bool InertiaController::fill_buffers(const ros::Duration& period)
{
  auto it = velocity_array_.begin();

  it = velocity_array_.begin();
  it = velocity_array_.insert(it, joint_.getVelocity());
  velocity_array_.resize(vel_size_);

  // Automatically fills the zero'th position of the acceleration array
  this->discrete_speed_derivative(period);
  acceleration_array_.resize(acc_size_);

  it = joint_torque_.begin();
  it = joint_torque_.insert(it, joint_.getEffort());
  joint_torque_.resize(torque_size_);
  return false;
}

void InertiaController::apply_butter()
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
void InertiaController::inertia_estimate()
{
  double K_i;
  double K_a;
  double torque_e;
  double acc_e;
  this->apply_butter();

  this->correlation_calculation();
  K_i = this->gain_calculation();
  K_a = this->alpha_calculation();
  torque_e = filtered_joint_torque_[0] - filtered_joint_torque_[1];
  acc_e = filtered_acceleration_array_[0] - filtered_acceleration_array_[1];
  joint_inertia_ = (torque_e - (acc_e * joint_inertia_)) * K_i * K_a + joint_inertia_;
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
double InertiaController::gain_calculation()
{
  return (corr_coeff_ * (filtered_acceleration_array_[0] - filtered_acceleration_array_[1])) /
         (lambda_ + corr_coeff_ * pow(filtered_acceleration_array_[0] - filtered_acceleration_array_[1], 2));
}

// Calculate the correlation coefficient of the acceleration buffer
void InertiaController::correlation_calculation()
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
double InertiaController::vibration_calculation()
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
void InertiaController::discrete_speed_derivative(const ros::Duration& period)
{
  auto it = acceleration_array_.begin();
  it = acceleration_array_.insert(it, (joint_.getVelocity() - velocity_array_[1]) / period.toSec());
}

}  // namespace joint_inertia_controller
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller::InertiaController, controller_interface::ControllerBase);
