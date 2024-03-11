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
#include "urdf/model.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

class TorqueConverter {
public:
    typedef std::unique_ptr<TorqueConverter> UniquePtr;
    typedef std::shared_ptr<TorqueConverter> SharedPtr;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

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

    void setInertialToBackpackOrientation(const Eigen::Quaterniond& orientation);

    Eigen::VectorXd rnea(
        const RobotNode::JointNameToValueMap& joint_positions, 
        const RobotNode::JointNameToValueMap& joint_velocities, 
        const RobotNode::JointNameToValueMap& joint_accelerations) const;

private:
    Eigen::VectorXd filterToeJointValues(const Eigen::VectorXd& joint_values) const;
    Eigen::VectorXd convertToEigenVector(const KDL::JntArray& kdl_jnt_array) const;
    Eigen::VectorXd convertToEigenVector(const RobotNode::JointNameToValueMap& joint_values) const;
    KDL::JntArray convertToKDLJntArray(const RobotNode::JointNameToValueMap& joint_values) const;
    RobotNode::JointNameToValueMap expandJointValuesWithToes(const RobotNode::JointNameToValueMap& joint_values) const;

    std::shared_ptr<RobotDescription> m_robot_description;
    std::vector<RobotNode::SharedPtr>
        m_joint_nodes; // TODO: This is a duplicate of m_robot_description->m_robot_node_ptrs
    std::unordered_map<std::string, RobotNode::SharedPtr> m_joint_nodes_map;
    std::unordered_map<std::string, RobotNode::EvaluateJacobianPtr> m_jacobian_position_map;

    urdf::Model m_urdf_model;
    KDL::Chain m_kdl_chain_left;
    KDL::Chain m_kdl_chain_right;

    std::vector<std::string> m_joint_names_left;
    std::vector<std::string> m_joint_names_right;

    Eigen::Vector3d m_gravity_vector;
    Eigen::Quaterniond m_inertial_to_backpack_orientation;
};

#endif // MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_