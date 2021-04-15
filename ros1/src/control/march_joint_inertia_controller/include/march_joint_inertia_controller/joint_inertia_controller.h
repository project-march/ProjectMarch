// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H
#define MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H

#include "march_joint_inertia_controller/inertia_estimator.h"
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>

namespace joint_inertia_controller {
class InertiaController : public controller_interface::Controller<
                              hardware_interface::EffortJointInterface> {
public:
    struct Commands {
        double position; // Last commanded position
        double velocity; // Last commanded velocity
        bool has_velocity; // false if no velocity command has been specified
    };

    InertiaController() = default;
    ~InertiaController() override;

    bool init(hardware_interface::EffortJointInterface* hw,
        ros::NodeHandle& n) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;
    void setCommand(double pos_command);
    void setCommand(double pos_command, double vel_command);
    void commandCB(const std_msgs::Float64ConstPtr& msg);

    /**
     * \brief Get the PID parameters
     */
    void getGains(
        double& p, double& i, double& d, double& i_max, double& i_min);
    /**
     * \brief Get the PID parameters
     */
    void getGains(double& p, double& i, double& d, double& i_max, double& i_min,
        bool& antiwindup);

    /**
     * \brief Print debug info to console
     */
    void printDebug();

    /**
     * \brief Set the PID parameters
     */
    void setGains(const double& p, const double& i, const double& d,
        const double& i_max, const double& i_min,
        const bool& antiwindup = false);

    /**
     * \brief Return the current position of the corresponding joint
     */
    double getPosition();

    /**
     * \brief Return the name of the corresponding joint is string format
     */
    std::string getJointName();

private:
    control_toolbox::Pid pid_controller_; /**< Internal PID controller. */

    ros::Subscriber sub_command_;
    hardware_interface::JointHandle joint_;

    urdf::JointConstSharedPtr joint_urdf_;
    std::string joint_name_;
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_; // pre-allocated memory that is re-used to set the
                              // realtime buffer

    InertiaEstimator inertia_estimator_;
};
} // namespace joint_inertia_controller

#endif // MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H
