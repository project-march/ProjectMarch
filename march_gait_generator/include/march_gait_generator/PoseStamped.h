// Copyright 2019 Project March

#ifndef MARCH_GAIT_GENERATOR_POSESTAMPED_H
#define MARCH_GAIT_GENERATOR_POSESTAMPED_H

#include <vector>
#include <string>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <march_gait_generator/Pose.h>

#include <ros/console.h>
#include <utility>

class PoseStamped
{
public:
  float seconds;
  float percentage;
  Pose pose;

  PoseStamped(float seconds, float percentage, const std::vector<std::string>& name);
  PoseStamped(float seconds, float percentage, Pose pose);
  explicit PoseStamped(const std::vector<std::string>& name);

  trajectory_msgs::JointTrajectoryPoint toJointTrajectoryPoint();

  /** @brief Override comparison operator */
  friend bool operator==(const PoseStamped& lhs, const PoseStamped& rhs)
  {
    return lhs.seconds == rhs.seconds && lhs.percentage == rhs.percentage;
  }

  friend bool operator!=(const PoseStamped& lhs, const PoseStamped& rhs)
  {
    return !(lhs == rhs);
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const PoseStamped& poseStamped)
  {
    return os << "seconds: " << poseStamped.seconds << " percentage: " << poseStamped.percentage;
  }
};

#endif  // MARCH_GAIT_GENERATOR_POSESTAMPED_H
