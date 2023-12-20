#ifndef MARCH_STATE_ESTIMATOR__ROBOT_JOINT_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_JOINT_HPP_

#include "march_state_estimator/robot_node.hpp"

class RobotJoint : public RobotNode
{
public:
    RobotJoint() = default;
    RobotJoint(const std::string & name, const std::vector<double> & axis);
    ~RobotJoint() = default;

private:
    std::vector<double> axis_;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_JOINT_HPP_