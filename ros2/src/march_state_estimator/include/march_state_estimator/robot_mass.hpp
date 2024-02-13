/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_MASS_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_MASS_HPP_

#include "march_state_estimator/robot_node.hpp"

class RobotMass : public RobotNode {
public:
    RobotMass(const std::string& name, const uint64_t& id, const double& mass);
    ~RobotMass() = default;
};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_MASS_HPP_