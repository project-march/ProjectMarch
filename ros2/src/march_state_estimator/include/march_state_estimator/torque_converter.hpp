/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_
#define MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_

#include <algorithm>
#include <functional>
#include <vector>

#include "march_state_estimator/robot_description.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

class TorqueConverter {
public:
    typedef std::unique_ptr<TorqueConverter> UniquePtr;
    typedef std::shared_ptr<TorqueConverter> SharedPtr;

    TorqueConverter(std::shared_ptr<RobotDescription> robot_description);
    ~TorqueConverter() = default;

    RobotNode::JointNameToValueMap getDynamicalJointAccelerations(
        const RobotNode::JointNameToValueMap& joint_positions, 
        const RobotNode::JointNameToValueMap& joint_torques) const;
    RobotNode::JointNameToValueMap getDynamicalTorques(
        const RobotNode::JointNameToValueMap& joint_positions,
        const RobotNode::JointNameToValueMap& joint_velocities, 
        const RobotNode::JointNameToValueMap& joint_accelerations) const;
    RobotNode::JointNameToValueMap getExternalTorques(
        const RobotNode::JointNameToValueMap& joint_total_torques, 
        const RobotNode::JointNameToValueMap& joint_dynamical_torques) const;
    Eigen::Vector3d getExternalForceByNode(const std::string& node_name, 
        const RobotNode::JointNameToValueMap& joint_positions, 
        const RobotNode::JointNameToValueMap& external_torques) const;
    std::vector<std::string> getJointNames() const;

private:
    Eigen::VectorXd convertToEigenVector(const RobotNode::JointNameToValueMap& joint_values) const;

    std::shared_ptr<RobotDescription> m_robot_description;
    std::vector<RobotNode::SharedPtr>
        m_joint_nodes; // TODO: This is a duplicate of m_robot_description->m_robot_node_ptrs
    std::unordered_map<std::string, RobotNode::SharedPtr> m_joint_nodes_map;
    std::unordered_map<std::string, RobotNode::EvaluateJacobianPtr> m_jacobian_position_map;
};

#endif // MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_