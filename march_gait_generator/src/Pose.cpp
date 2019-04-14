// Copyright 2019 Project March

#include <march_gait_generator/Pose.h>

Pose::Pose(const std::vector<std::string>& name)
{
  sensor_msgs::JointState jointState;
  for (int i = 0; i < name.size(); i++)
  {
    jointState.name.push_back(name.at(i));
    jointState.position.push_back(0);
    jointState.velocity.push_back(0);
  }
  this->fromJointState(jointState);
}

Pose::Pose(const std::vector<std::string>& name, const std::vector<double>& position,
           const std::vector<double>& velocity)
  : name(name), position(position), velocity(velocity)
{
}

Pose::Pose(const sensor_msgs::JointState& jointState)
  : name(jointState.name), position(jointState.position), velocity(jointState.velocity)
{
}

void Pose::fromJointState(const sensor_msgs::JointState& jointState)
{
  this->name = jointState.name;
  this->position = jointState.position;
  this->velocity = jointState.velocity;
}

sensor_msgs::JointState Pose::toJointState()
{
  sensor_msgs::JointState jointState;
  jointState.name = this->name;
  jointState.position = this->position;
  jointState.velocity = this->velocity;
  return jointState;
}

std::map<std::string, double> Pose::toPositionMap()
{
  std::map<std::string, double> map;
  for (int i = 0; i < this->name.size(); i++)
  {
    std::pair<std::string, double> entry;
    entry.first = this->name.at(i);
    entry.second = this->position.at(i);
    map.insert(entry);
  }
  return map;
}

int Pose::getJointIndex(std::string jointName)
{
  int index = -1;
  auto it = std::find(name.begin(), name.end(), jointName);
  if (it != name.end())
  {
    index = std::distance(name.begin(), it);
  }
  else
  {
    ROS_WARN("Joint %s does not exist in this pose", jointName.c_str());
  }

  return index;
}

double Pose::getJointPosition(std::string jointName)
{
  int index = this->getJointIndex(std::move(jointName));
  if (index == -1)
  {
    return 0;
  }
  return this->position.at(index);
}

double Pose::getJointVelocity(std::string jointName)
{
  int index = this->getJointIndex(std::move(jointName));
  if (index == -1)
  {
    return 0;
  }
  return this->velocity.at(index);
}

void Pose::setJointPosition(std::string jointName, float position)
{
  int index = this->getJointIndex(std::move(jointName));
  if (index != -1)
  {
    this->position.at(index) = position;
  }
}

void Pose::setJointVelocity(std::string jointName, float velocity)
{
  int index = this->getJointIndex(std::move(jointName));
  if (index != -1)
  {
    this->velocity.at(index) = velocity;
  }
}
