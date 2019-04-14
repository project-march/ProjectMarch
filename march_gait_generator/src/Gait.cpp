// Copyright 2019 Project March

#include <march_gait_generator/Gait.h>

Gait::Gait() : Gait("Default", "Description", "0.1")
{
}

Gait::Gait(const std::string& name, const std::string& comment, const std::string& version,
           const ros::Duration& duration)
  : name(name), comment(comment), version(version), duration(duration), poseList({})
{
}

Gait::Gait(const std::string& name, const std::string& comment, const std::string& version)
  : name(name), comment(comment), version(version), duration(ros::Duration(10))
{
}

trajectory_msgs::JointTrajectory Gait::toJointTrajectory()
{
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = this->poseList.at(0).pose.name;
  for (int i = 0; i < this->poseList.size(); i++)
  {
    trajectory.points.push_back(this->poseList.at(i).toJointTrajectoryPoint());
  }
  return trajectory;
}

void Gait::addPoseStamped(int index, PoseStamped poseStamped)
{
  poseList.insert(poseList.begin() + index, poseStamped);
}

void Gait::addPoseStamped(PoseStamped poseStamped)
{
  poseList.push_back(poseStamped);
}
