
#ifndef MARCH_GAIT_GENERATOR_POSESTAMP_H
#define MARCH_GAIT_GENERATOR_POSESTAMP_H


#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <utility>

class PoseStamp {

private:
    float seconds;
    float percentage;

public:

    PoseStamp(float seconds, float percentage) : seconds(seconds), percentage(percentage) {}

    /** @brief Override comparison operator */
    friend bool operator==(const PoseStamp& lhs, const PoseStamp& rhs)
    {
        return lhs.seconds == rhs.seconds && lhs.percentage == rhs.percentage;
    }

    friend bool operator!=(const PoseStamp& lhs, const PoseStamp& rhs)
    {
        return !(lhs == rhs);
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(std::ostream& os, const PoseStamp& poseStamp)
    {
        return os << "seconds: " << poseStamp.seconds << " percentage: " << poseStamp.percentage;
    }


};


#endif //MARCH_GAIT_GENERATOR_PoseStamp_H
