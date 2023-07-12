#ifndef WEIGHT_SHIFT_BUFFER__HPP
#define WEIGHT_SHIFT_BUFFER__HPP

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cstdio>

class WeightShiftBuffer {
public:
    WeightShiftBuffer();
    void update_HAA_during_step();
    void add_weight_shift();
    void reset_HAA_at_end();
    void fix_timings_traj();
    trajectory_msgs::msg::JointTrajectory return_final_traj_with_weight_shift(trajectory_msgs::msg::JointTrajectory);

    void set_incoming_joint_trajectory(trajectory_msgs::msg::JointTrajectory);

private:
    trajectory_msgs::msg::JointTrajectory m_incoming_joint_trajectory; 
    trajectory_msgs::msg::JointTrajectoryPoint m_first_traj_point;

    trajectory_msgs::msg::JointTrajectory m_final_joint_trajectory;
    
    double m_duration_weight_shift;
    double m_duration_step;
    double m_duration_reset_haa;

    float m_hip_aa_position; //position HAA's go to during weight_shift

    std::string swing_leg; //which leg does the step


};

#endif