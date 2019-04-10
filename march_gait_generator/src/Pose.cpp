//
// Created by ishadijcks on 10-4-19.
//

#include <march_gait_generator/Pose.h>

Pose::Pose(const std::vector<std::string> &name, const std::vector<double> &position,
           const std::vector<double> &velocity) : name(name), position(position), velocity(velocity) {}

Pose::Pose(const sensor_msgs::JointState& jointState) :name (jointState.name), position(jointState.position), velocity(jointState.velocity){

}
