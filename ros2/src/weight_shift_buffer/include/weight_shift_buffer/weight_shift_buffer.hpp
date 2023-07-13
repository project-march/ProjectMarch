#ifndef WEIGHT_SHIFT_BUFFER__HPP
#define WEIGHT_SHIFT_BUFFER__HPP

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cstdio>
#include <math.h>

class WeightShiftBuffer {
public:
    WeightShiftBuffer();

    void set_incoming_joint_trajectory(trajectory_msgs::msg::JointTrajectory);
    void update_HAA_during_step();
    void add_weight_shift();
    void reset_HAA_at_end();
    void fix_timings_traj();

    // weight shift before
    trajectory_msgs::msg::JointTrajectory return_final_traj_with_weight_shift(trajectory_msgs::msg::JointTrajectory);
    
    std::string swing_leg; // which leg does the step

    // weight shift during
    trajectory_msgs::msg::JointTrajectory add_weight_shift_during_gait();

    // For Koen's asymmetry, we try to make the right step trajectory more extreme
    // as we see he has more trouble making a nice step with his right leg.
    trajectory_msgs::msg::JointTrajectory fix_asymmetry(trajectory_msgs::msg::JointTrajectory);
    
    // dynamic parameter tuning
    void set_weight_shift_duration(double);
    void set_step_size(double);
    void set_swing_scaling(double);

private:
    trajectory_msgs::msg::JointTrajectory m_incoming_joint_trajectory;
    trajectory_msgs::msg::JointTrajectoryPoint m_first_traj_point;

    trajectory_msgs::msg::JointTrajectory m_final_joint_trajectory;

    double m_duration_weight_shift;
    double m_duration_step;
    double m_duration_reset_haa;
    double m_right_swing_scaling;

    float m_hip_aa_position; // position HAA's go to during weight_shift

};

#endif