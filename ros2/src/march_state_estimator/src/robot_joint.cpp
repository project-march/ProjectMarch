#include "march_state_estimator/robot_joint.hpp"

RobotJoint::RobotJoint(const std::string & name, const uint64_t & id, const std::vector<double> & axis)
{
    name_ = name;
    id_ = id;
    type_ = 'J';

    axis_ = axis;
}