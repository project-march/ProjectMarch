
#ifndef MARCH_GAIT_GENERATOR_GAIT_H
#define MARCH_GAIT_GENERATOR_GAIT_H


#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <utility>
#include <march_gait_generator/Pose.h>
#include <march_gait_generator/PoseStamped.h>

class Gait {

private:


public:
    std::string name;
    std::string comment;
    std::string version;
    ros::Duration duration;
    std::vector<PoseStamped> poseList;

    Gait();
    Gait(const std::string &name, const std::string &comment, const std::string &version, const ros::Duration &duration);
    Gait(const std::string &name, const std::string &comment, const std::string &version);

    /**
     * @brief Add a PoseStamped add a specific index in the poseList.
     */
    void addPoseStamped(int index, PoseStamped poseStamped);

    /**
     * @brief Add a PoseStamped to the end of the poseList.
     */
    void addPoseStamped(PoseStamped poseStamped);

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
