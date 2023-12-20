#include "march_state_estimator/robot_joint.hpp"

RobotJoint::RobotJoint(const std::string & name, const std::vector<double> & axis)
{
    name_ = name;
    axis_ = axis;
}