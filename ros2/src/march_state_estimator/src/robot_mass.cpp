/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_mass.hpp"

RobotMass::RobotMass(const std::string& name, const uint64_t& id, const double& mass)
{
    m_name = name;
    m_id = id;
    m_type = 'M';

    m_mass = mass;
}