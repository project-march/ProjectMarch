// Copyright 2019 Project March.

#include <gazebo/physics/physics.hh>

#ifndef MARCH_GAZEBO_PLUGINS_OBSTACLE_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_OBSTACLE_CONTROLLER_H

namespace gazebo
{
class ObstacleController
{
public:
  explicit ObstacleController(physics::ModelPtr model);

  void newSubgait(const march_shared_resources::GaitActionGoalConstPtr& _msg);
  double getMass();
  ignition::math::v4::Vector3<double> GetCom();
  virtual void update(ignition::math::v4::Vector3<double>& torque_all,
                      ignition::math::v4::Vector3<double>& torque_stable) = 0;

private:
  physics::ModelPtr model_;

  physics::LinkPtr foot_left;
  physics::LinkPtr foot_right;

  double subgait_start_time;
  double swing_step_size;
  double subgait_duration;
};
}  // namespace gazebo

#endif  // MARCH_RQT_GAIT_GENERATOR_OBSTACLECONTROLLER_H
