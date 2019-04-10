
#ifndef MARCH_GAIT_GENERATOR_GAIT_H
#define MARCH_GAIT_GENERATOR_GAIT_H


#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <utility>
#include <march_gait_generator/Pose.h>
#include <march_gait_generator/PoseStamp.h>

class Gait {

private:
    std::string name;
    std::string comment;
    std::string version;
    ros::Duration duration;
    std::vector<std::pair<PoseStamp, Pose>> poseList;

public:

    Gait(const std::string &name, const std::string &comment, const std::string &version, const ros::Duration &duration);

    /** @brief Override comparison operator */
    friend bool operator==(const Gait& lhs, const Gait& rhs)
    {
        return lhs.name == rhs.name && lhs.comment == rhs.comment && lhs.version == rhs.version &&
        lhs.duration.toSec() == rhs.duration.toSec();
    }

    friend bool operator!=(const Gait& lhs, const Gait& rhs)
    {
        return !(lhs == rhs);
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(std::ostream& os, const Gait& gait)
    {
        return os << "name: " << gait.name << " comment: " << gait.comment << " version: " << gait.version << " duration: " << gait.duration.toSec();
    }


};


#endif //MARCH_GAIT_GENERATOR_GAIT_H
