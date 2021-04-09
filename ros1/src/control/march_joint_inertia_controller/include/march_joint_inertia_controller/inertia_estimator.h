// Copyright 2020 Project March.

#ifndef MARCH_WS_INERTIA_ESTIMATOR_H
#define MARCH_WS_INERTIA_ESTIMATOR_H

#include <hardware_interface/joint_command_interface.h>
#include <joint_trajectory_controller/trajectory_interface/quintic_spline_segment.h>
#include <list>
#include <ros/ros.h>
#include <urdf/model.h>

/**
 * \brief determines the absolute value of vector a and stores it into b so that
 * a remains unchanged
 *
 * @param[in] a input vector with values that need to be converted to absolutes
 * @param[out] b output vector with copies of the absolute values of a.
 */
std::list<double> absolute(const std::list<double>& a);

/**
 * \brief determiness the mean value of the given vector
 *
 * @param[in] a input vector from which the mean gets calculated
 * @returns the mean value of a
 */
double mean(const std::list<double>& a);

/**
 * \brief Class that bundles functionality to estimate inertia on a Revolute
 * Joint.
 *
 * @param[in] joint the jointhandle used for obtaining current speed and effort.
 * @param[in] lambda sets the lambda used in the RLS filter defaulted to 0.96
 * @param[in] acc_size the size of the vector wherein the calculated
 * acceleration is stored. This vector length is also used in determining the
 * length of the velocity vector and the filtered acceleration vector. It is of
 * specific size to calculate the alpha value. Defaulted to 12.
 */
class InertiaEstimator {
public:
    /**
     * \brief Default constructor
     */
    InertiaEstimator(double lambda = 0.96, size_t acc_size = 12);

    double getAcceleration();
    double getJointInertia();

    void setLambda(double lambda);
    void setAccSize(size_t acc_size);
    void setVibrationBoundaries(std::vector<double> boundaries);

    /**
     * \brief Applies the Butterworth filter over the last two samples and
     * returns the resulting filtered value
     */
    void applyButter();

    /**
     * \brief Estimate the inertia using the acceleration and torque
     */
    void inertiaEstimate();
    /**
     * \brief Calculate a discrete derivative of the speed measurements
     */
    double discreteSpeedDerivative(const ros::Duration& period);
    /**
     * \brief Calculate the alpha coefficient for the inertia estimate
     */
    double alphaCalculation();
    /**
     * \brief Calculate the inertia gain for the inertia estimate
     */
    double gainCalculation();
    /**
     * \brief Calculate the correlation coefficient of the acceleration buffer
     */
    void correlationCalculation();
    /**
     * \brief Calculate the vibration based on the acceleration
     */
    double vibrationCalculation();

    /**
     * \brief Fill the rolling buffers with corresponding values.
     */
    void fillBuffers(
        double velocity, double effort, const ros::Duration& period);

    /**
     * \brief Calculate the initial correlation coefficient
     */
    void initP(unsigned int samples);

    // Vector to be filled with samples of acceleration to determine the
    // standard deviation from
    std::list<double> standard_deviation;

private:
    double min_alpha_; // You might want to be able to adjust this value from a
                       // yaml/launch file
    double max_alpha_; // You might want to be able to adjust this value from a
                       // yaml/launch file

    // This is a sixth order butterworth filter with a cutoff frequency at 15Hz
    // in Second Order Sections form
    std::vector<std::vector<double>> sos_ {
        { 9.16782507e-09, 1.83356501e-08, 9.16782507e-09, 1.00000000e+00,
            -1.82520938e+00, 8.33345838e-01 },
        { 1.00000000e+00, 2.00000000e+00, 1.00000000e+00, 1.00000000e+00,
            -1.86689228e+00, 8.75214548e-01 },
        { 1.00000000e+00, 2.00000000e+00, 1.00000000e+00, 1.00000000e+00,
            -1.94377925e+00, 9.52444269e-01 }
    };

    // z1 and z2 vectors are containers for previous time-step values in the
    // butterworth filter Might be neater to design a separate class for the
    // filter application for arbitrary filter sizes, datasets and number of
    // signals such as:
    // https://github.com/scipy/scipy/blob/v1.5.2/scipy/signal/_sosfilt.pyx
    std::vector<double> z1a;
    std::vector<double> z2a;

    std::vector<double> z1t;
    std::vector<double> z2t;

    // Default length 12 for the alpha calculation
    size_t acc_size_ = 12;
    std::list<double> acceleration_array_;
    // Equal to two, because two values are needed to calculate the derivative
    size_t vel_size_ = acc_size_;
    std::list<double> velocity_array_;
    std::list<double> filtered_acceleration_array_;
    // Of length 3 for the butterworth filter
    size_t torque_size_ = 3;
    std::list<double> joint_torque_;
    // Of length 2 for the error calculation
    size_t fil_tor_size_ = 2;
    std::list<double> filtered_joint_torque_;

    // Correlation coefficient used to calculate the inertia gain
    double corr_coeff_;
    double vibration_;
    double K_a_;
    double K_i_;
    double mean_of_absolute_;
    double absolute_of_mean_;
    double joint_inertia_;
    double lambda_;
};

#endif // MARCH_WS_INERTIA_ESTIMATOR_H
