
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

    int getJointIndex(std::string jointName);

public:
    Pose(const std::vector<std::string> &name, const std::vector<double> &position,
         const std::vector<double> &velocity);

    Pose(const sensor_msgs::JointState& jointState);

    Pose(const std::vector<std::string> &name);

    double getJointPosition(std::string jointName);
    double getJointVelocity(std::string jointName);

    void fromJointState(const sensor_msgs::JointState& jointState){
        this->name = jointState.name;
        this->position = jointState.position;
        this->velocity = jointState.velocity;
    }

    sensor_msgs::JointState toJointState(){
        sensor_msgs::JointState jointState;
        jointState.name = this->name;
        jointState.position = this->position;
        jointState.velocity = this->velocity;
        return jointState;
    }

    /** @brief Override comparison operator */
    friend bool operator==(const Pose& lhs, const Pose& rhs)
    {
        return lhs.name == rhs.name && lhs.position == rhs.position && lhs.velocity == rhs.velocity;
    }

    friend bool operator!=(const Pose& lhs, const Pose& rhs)
    {
        return !(lhs == rhs);
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(std::ostream& os, const Pose& pose)
    {

        std::stringstream nameStream;
        nameStream << "names: {";
        std::copy(pose.name.begin(), pose.name.end(), std::ostream_iterator<std::string>(nameStream, " "));
        nameStream << "} ";

        std::stringstream positionStream;
        positionStream << "positions: {";
        std::copy(pose.position.begin(), pose.position.end(), std::ostream_iterator<double>(positionStream, " "));
        positionStream << "} ";

        std::stringstream velocityStream;
        velocityStream << "velocities: {";
        std::copy(pose.velocity.begin(), pose.velocity.end(), std::ostream_iterator<double>(velocityStream, " "));
        velocityStream << "} ";

        return os << nameStream.str() << positionStream.str() << velocityStream.str();
    }


};


#endif //MARCH_GAIT_GENERATOR_POSE_H
