#include "march_state_estimator/robot_mass.hpp"

RobotMass::RobotMass(const std::string & name, const double & mass)
{
    name_ = name;
    mass_ = mass;
}