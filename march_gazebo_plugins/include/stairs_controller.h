// Copyright 2019 Project March.

#include <obstacle_controller.h>

#ifndef MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H

namespace gazebo
{
class StairsController : public ObstacleController
{
public:
  explicit StairsController(physics::ModelPtr model);
  void update(ignition::math::v4::Vector3<double>& torque_all, ignition::math::v4::Vector3<double>& torque_stable);

private:
  double p_yaw;
  double d_yaw;
  double p_pitch;
  double d_pitch;
  double p_roll;
  double d_roll;
  double error_x_last_timestep;
  double error_y_last_timestep;
  double error_yaw_last_timestep;
};
}  // namespace gazebo

#endif  // MARCH_RQT_GAIT_GENERATOR_OBSTACLECONTROLLER_H
