// Copyright 2019 Project March.

#include <march_gait_generator/PoseStamped.h>

PoseStamped::PoseStamped(float seconds, float percentage, const std::vector<std::string>& name)
        : seconds(seconds), percentage(percentage), pose(name)
{
}

PoseStamped::PoseStamped(float seconds, float percentage, Pose pose) : seconds(seconds), percentage(percentage), pose(pose)
{
}

PoseStamped::PoseStamped(const std::vector<std::string>& name) : PoseStamped(0, 0, name)
{
}

trajectory_msgs::JointTrajectoryPoint PoseStamped::toJointTrajectoryPoint()
{
    trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint;
    jointTrajectoryPoint.positions = this->pose.position;
    jointTrajectoryPoint.velocities = this->pose.velocity;
    jointTrajectoryPoint.time_from_start = ros::Duration(this->seconds);
    return jointTrajectoryPoint;
}