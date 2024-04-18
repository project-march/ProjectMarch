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
#include "kdl/chain.hpp"

class TorqueConverter {
public:
    typedef std::unique_ptr<TorqueConverter> UniquePtr;
    typedef std::shared_ptr<TorqueConverter> SharedPtr;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    TorqueConverter(std::shared_ptr<RobotDescription> robot_description, const std::string& urdf_file_path);
    ~TorqueConverter() = default;

    // TODO: Create an update method that updates the joint states
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
    RobotNode::JointNameToValueMap getTotalTorques(
        const RobotNode::JointNameToValueMap& joint_external_torques, 
        const RobotNode::JointNameToValueMap& joint_dynamical_torques) const;
    Eigen::Vector3d getExternalForceByNode(const std::string& node_name, 
        const RobotNode::JointNameToValueMap& joint_positions, 
        const RobotNode::JointNameToValueMap& external_torques) const;
    std::vector<Eigen::Vector3d> getWorldTorqueInLegs(
        const RobotNode::JointNameToValueMap& joint_positions,
        const RobotNode::JointNameToValueMap& joint_torques) const;
    std::vector<std::string> getJointNames() const;
    std::vector<Eigen::Vector3d> convertTorqueToForce(
        const RobotNode::JointNameToValueMap& joint_positions,
        const RobotNode::JointNameToValueMap& joint_torques) const;

    inline void setInertialOrientation(const Eigen::Quaterniond& orientation) { m_world_to_backpack_orientation = orientation; }

private:
    Eigen::VectorXd convertJointNameToValueMapToEigenVector(const RobotNode::JointNameToValueMap& joint_values) const;
    RobotNode::JointNameToValueMap recursiveNewtonEuler(
        const RobotNode::JointNameToValueMap& joint_positions,
        const RobotNode::JointNameToValueMap& joint_velocities,
        const RobotNode::JointNameToValueMap& joint_accelerations) const;
    std::unordered_map<std::string, Eigen::Vector3d> vectorizeJointTorqueVectors(const RobotNode::JointNameToValueMap& joint_torques) const;
    std::unordered_map<std::string, Eigen::Vector3d> orientateTorqueVectorsToWorld(
        const RobotNode::JointNameToValueMap& joint_positions,
        const std::unordered_map<std::string, Eigen::Vector3d>& joint_torque_vectors) const;
    std::unordered_map<std::string, Eigen::Vector3d> orientateTorqueVectorsToFoot(
        const RobotNode::JointNameToValueMap& joint_positions,
        const std::unordered_map<std::string, Eigen::Vector3d>& joint_torque_vectors) const;

    std::shared_ptr<RobotDescription> m_robot_description;
    std::vector<RobotNode::SharedPtr>
        m_joint_nodes; // TODO: This is a duplicate of m_robot_description->m_robot_node_ptrs
    std::unordered_map<std::string, RobotNode::SharedPtr> m_joint_nodes_map;
    std::unordered_map<std::string, RobotNode::EvaluateJacobianPtr> m_jacobian_position_map;
    Eigen::Quaterniond m_world_to_backpack_orientation;

    KDL::Chain m_kdl_chain_leg_left;
    KDL::Chain m_kdl_chain_leg_right;
    const Eigen::Vector3d m_world_gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
};

#endif // MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_