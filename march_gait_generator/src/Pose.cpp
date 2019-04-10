//
// Created by ishadijcks on 10-4-19.
//

#include <march_gait_generator/Pose.h>


Pose::Pose(const std::vector<std::string> &name) {
    sensor_msgs::JointState jointState;
    for(int i = 0; i<name.size(); i++){
        jointState.name.push_back(name.at(i));
    }
    this->fromJointState(jointState);
}


Pose::Pose(const std::vector<std::string> &name, const std::vector<double> &position,
           const std::vector<double> &velocity) : name(name), position(position), velocity(velocity) {}

Pose::Pose(const sensor_msgs::JointState& jointState) :name (jointState.name), position(jointState.position), velocity(jointState.velocity){

}

int Pose::getJointIndex(std::string jointName){
    int index = -1;
    auto it = std::find(name.begin(), name.end(), jointName);
    if (it != name.end()) {
        index = std::distance(name.begin(), it);
    }
    else {
        ROS_WARN("Joint %s does not exist in this pose", jointName.c_str());
    }

    return index;
}

double Pose::getJointPosition(std::string jointName){
    int index = this->getJointIndex(std::move(jointName));
    if (index == -1){
        return 0;
    }
    return this->position.at(index);
}

double Pose::getJointVelocity(std::string jointName){
    int index = this->getJointIndex(std::move(jointName));
    if (index == -1){
        return 0;
    }
    return this->velocity.at(index);}
