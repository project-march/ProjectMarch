// Copyright 2020 Project March.
#include "march_joint_inertia_controller/inertia_estimator.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <algorithm>
#include <cmath>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

std::list<double> absolute(const std::list<double>& a)
{
    std::list<double> b;
    for (auto it = a.begin(); it != a.end(); it++) {
        b.push_back(abs(*it));
    }
    return b;
}

double mean(const std::list<double>& a)
{
    double sum = 0.0;
    for (const auto& it : a) {
        sum += it;
    }
    if (a.size() == 0) {
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

double InertiaEstimator::getAcceleration()
{
    return acceleration_array_.front();
}

void InertiaEstimator::setLambda(double lambda)
{
    lambda_ = lambda;
}
void InertiaEstimator::setAccSize(size_t acc_size)
{
    acc_size_ = acc_size;

    velocity_array_.resize(acc_size_, 0.0);
    acceleration_array_.resize(acc_size_, 0.0);
    filtered_acceleration_array_.resize(acc_size_, 0.0);
}

void InertiaEstimator::setVibrationBoundaries(std::vector<double> boundaries)
{
    min_alpha_ = boundaries[0];
    max_alpha_ = boundaries[1];
}

double InertiaEstimator::getJointInertia()
{
    return joint_inertia_;
}

// Fills the buffers so that non-zero values may be computed by the inertia
// estimator
void InertiaEstimator::fillBuffers(
    double velocity, double effort, const ros::Duration& period)
{
    velocity_array_.push_front(velocity);
    velocity_array_.pop_back();

    // Automatically fills the zero'th position of the acceleration array
    double acc = discreteSpeedDerivative(period);
    acceleration_array_.push_front(acc);
    acceleration_array_.pop_back();

    joint_torque_.push_front(effort);
    joint_torque_.pop_back();
}

void InertiaEstimator::applyButter()
{
    // Apply a sixth order Butterworth filter over the effort and acceleration
    // signals
    int i = 0;
    double x = 0.0;
    double x_n = 0.0;

    for (const auto& so : sos_) {
        x_n = acceleration_array_.front();
        x = so[0] * *acceleration_array_.begin() + z1a[i];
        z1a[i] = so[1] * x_n - so[4] * x + z2a[i];
        z2a[i] = so[2] * x_n - so[5] * x;
        i++;
    }

    filtered_acceleration_array_.push_front(x);
    filtered_acceleration_array_.pop_back();

    i = 0;
    for (const auto& so : sos_) {
        x_n = *joint_torque_.begin();
        x = so[0] * *joint_torque_.begin() + z1t[i];
        z1t[i] = so[1] * x_n - so[4] * x + z2t[i];
        z2t[i] = so[2] * x_n - so[5] * x;
        i++;
    }
    filtered_joint_torque_.push_front(x);
    filtered_joint_torque_.pop_back();
}

// Estimate the inertia using the acceleration and torque
void InertiaEstimator::inertiaEstimate()
{
    applyButter();

    correlationCalculation();
    K_i_ = gainCalculation();
    K_a_ = alphaCalculation();
    auto ita = filtered_acceleration_array_.begin();
    auto itt = filtered_joint_torque_.begin();
    const double torque_e = *itt - *(++itt);
    const double acc_e = *ita - *(++ita);
    joint_inertia_
        = (torque_e - (acc_e * joint_inertia_)) * K_i_ * K_a_ + joint_inertia_;
}

// Calculate the alpha coefficient for the inertia estimate
double InertiaEstimator::alphaCalculation()
{
    double vib
        = std::min(std::max(vibrationCalculation(), min_alpha_), max_alpha_);
    vibration_ = (vib - min_alpha_) / (max_alpha_ - min_alpha_);
    return vibration_;
}

// Calculate the inertia gain for the inertia estimate
double InertiaEstimator::gainCalculation()
{
    auto it = filtered_acceleration_array_.begin();
    auto error = *it - *(++it);
    return (corr_coeff_ * error) / (lambda_ + corr_coeff_ * pow(error, 2));
}

// Calculate the correlation coefficient of the acceleration buffer
void InertiaEstimator::correlationCalculation()
{
    auto it = filtered_acceleration_array_.begin();
    corr_coeff_ = corr_coeff_ / (lambda_ + corr_coeff_ * pow(*it - *(++it), 2));
    const double large_number = 10e8;
    if (corr_coeff_ > large_number) {
        corr_coeff_ = large_number;
    }
}

// Calculate the vibration based on the acceleration
double InertiaEstimator::vibrationCalculation()
{
    mean_of_absolute_ = mean(absolute(filtered_acceleration_array_));
    absolute_of_mean_ = abs(mean(filtered_acceleration_array_));
    // Divide by zero protection, necessary when there has not been any
    // acceleration yet
    if (absolute_of_mean_ == 0.0) {
        return 0.0;
    }
    return mean_of_absolute_ / absolute_of_mean_;
}

// Calculate a discrete derivative of the speed measurements
double InertiaEstimator::discreteSpeedDerivative(const ros::Duration& period)
{
    auto it = velocity_array_.begin();
    return (*it - *(++it)) / period.toSec();
}

void InertiaEstimator::initP(unsigned int samples)
{
    // Setup the initial value for the correlation coefficient
    // 100*standarddeviation(acceleration)^2
    double mean_value = mean(standard_deviation);
    double sum = 0;

    for (const auto& it : standard_deviation) {
        sum += std::pow(it - mean_value, 2);
    }
    corr_coeff_ = 100 * (sum / samples);
}
