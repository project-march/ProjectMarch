// Copyright 2019 Project March.
#ifndef MARCH_GAZEBO_PLUGINS_OBSTACLE_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_OBSTACLE_CONTROLLER_H

#include "yaml-cpp/yaml.h"
#include <gazebo/physics/physics.hh>
#include <march_shared_msgs/CurrentGait.h>

namespace gazebo {
class ObstacleController {
public:
    explicit ObstacleController(physics::ModelPtr model);

    void newSubgait(const march_shared_msgs::CurrentGaitConstPtr& msg);
    ignition::math::v6::Vector3<double> GetCom();
    void update(ignition::math::v6::Vector3<double>& torque_all,
        ignition::math::v6::Vector3<double>& torque_stable);
    void getGoalPosition(double time_since_start);
    void getSitGoalPositionX(
        double time_since_start, double stable_foot_pose_x);
    void getStandGoalPositionX(
        double time_since_start, double stable_foot_pose_x);
    void getWalkGoalPositionX(
        double time_since_start, double stable_foot_pose_x);
    bool changeComLevel(const std::string&);

    std::vector<std::string> com_levels;

protected:
    physics::ModelPtr model_;

    physics::LinkPtr foot_left_;
    physics::LinkPtr foot_right_;
    double halved_upper_leg_length_;
    double mass;

    std::string HOME_STAND;
    std::string STAND_IDLE;
    std::string SIT_IDLE;
    std::string default_subgait_name_;
    std::string subgait_name_;
    double subgait_start_time_;
    double swing_step_size_ {};
    double subgait_duration_;

    bool subgait_changed_;
    bool balance_;

    double p_yaw_;
    double d_yaw_;
    double p_yaw_balance_;
    double d_yaw_balance_;

    double p_pitch_;
    double d_pitch_;
    double p_pitch_balance_;
    double d_pitch_balance_;

    double p_roll_;
    double d_roll_;
    double p_roll_balance_;
    double d_roll_balance_;

    double error_x_last_timestep_;
    double error_y_last_timestep_;
    double error_yaw_last_timestep_;

    double goal_position_x;
    double goal_position_y;

    YAML::Node com_levels_tree;
};
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_OBSTACLE_CONTROLLER_H
