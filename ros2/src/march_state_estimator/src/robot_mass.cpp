#include "march_state_estimator/robot_mass.hpp"

RobotMass::RobotMass(const std::string & name, const uint64_t & id, const double & mass)
{
    name_ = name;
    id_ = id;
    type_ = 'M';

    mass_ = mass;
}