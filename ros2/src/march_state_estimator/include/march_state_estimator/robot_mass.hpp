#ifndef MARCH_STATE_ESTIMATOR__ROBOT_MASS_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_MASS_HPP_

#include "march_state_estimator/robot_node.hpp"

class RobotMass : public RobotNode
{
public:
    RobotMass() = default;
    RobotMass(const std::string & name, const double & mass);
    ~RobotMass() = default;

};

#endif  // MARCH_STATE_ESTIMATOR__ROBOT_MASS_HPP_