
#ifndef MARCH_GAIT_GENERATOR_POSE_H
#define MARCH_GAIT_GENERATOR_POSE_H


#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <utility>

class Pose {

private:
    std::vector<std::string> name;
    std::vector<double> position;
    std::vector<double> velocity;

    int getJointIndex(std::string jointName){
        int index = -1;
        auto it = std::find(name.begin(), name.end(), jointName);
        if (it != name.end()) {
            std::cout << "Element Found" << std::endl;
            index = std::distance(name.begin(), it);
        }
        else {
            ROS_WARN("Joint %s does not exist in this pose", jointName.c_str());
        }

        return index;
    }



public:
    Pose(const std::vector<std::string> &name, const std::vector<double> &position,
         const std::vector<double> &velocity);

    Pose(const sensor_msgs::JointState& jointState);


    double getJointPosition(std::string jointName){
        return this->position.at(this->getJointIndex(std::move(jointName)));
    }
};


#endif //MARCH_GAIT_GENERATOR_POSE_H
