// Copyright 2019 Project March.

#include <obstacle_controller.h>

#ifndef MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H

namespace gazebo
{
class WalkController : public ObstacleController
{
public:
  //  explicit WalkController();
  explicit WalkController(physics::ModelPtr model);
  void update(ignition::math::v4::Vector3<double>& torque_all, ignition::math::v4::Vector3<double>& torque_stable);

private:
  double p_yaw_;
  double d_yaw_;
  double p_pitch_;
  double d_pitch_;
  double p_roll_;
  double d_roll_;
  double error_x_last_timestep_;
  double error_y_last_timestep_;
  double error_yaw_last_timestep_;
};
}  // namespace gazebo

#endif  // MARCH_RQT_GAIT_GENERATOR_OBSTACLECONTROLLER_H
