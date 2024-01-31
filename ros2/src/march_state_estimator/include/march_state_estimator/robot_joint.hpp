/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_JOINT_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_JOINT_HPP_

#define JOINT_TYPE_REVOLUTE 1
#define JOINT_TYPE_CONTINUOUS 2
#define JOINT_TYPE_PRISMATIC 3
#define JOINT_TYPE_FLOATING 4
#define JOINT_TYPE_PLANAR 5
#define JOINT_TYPE_FIXED 6

#include "march_state_estimator/robot_node.hpp"

class RobotJoint : public RobotNode {
public:
    RobotJoint(const std::string& name, const uint64_t& id, const std::vector<double>& axis);
    ~RobotJoint() = default;

    void setLimits(const double& lower_limit, const double& upper_limit);
    Eigen::MatrixXd getGlobalRotationJacobian(JointNameToValueMap joint_positions) const;

private:
    double m_lower_limit;
    double m_upper_limit;
};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_JOINT_HPP_