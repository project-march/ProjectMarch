/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_
#define MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_

#include <algorithm>
#include <functional>
#include <vector>
#include <unordered_map>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "kdl/chain.hpp"

class TorqueConverter {
public:
    typedef std::unique_ptr<TorqueConverter> UniquePtr;
    typedef std::shared_ptr<TorqueConverter> SharedPtr;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef std::unordered_map<std::string, double> JointNameToValueMap;

    TorqueConverter(std::string& urdf_file_path);
    ~TorqueConverter() = default;

    // TODO: Create an update method that updates the joint states
    JointNameToValueMap getDynamicalJointAccelerations(
        const JointNameToValueMap& joint_positions, 
        const JointNameToValueMap& joint_torques) const;
    JointNameToValueMap getDynamicalTorques(
        const JointNameToValueMap& joint_positions,
        const JointNameToValueMap& joint_velocities, 
        const JointNameToValueMap& joint_accelerations) const;
    JointNameToValueMap getExternalTorques(
        const JointNameToValueMap& joint_total_torques, 
        const JointNameToValueMap& joint_dynamical_torques) const;
    JointNameToValueMap getTotalTorques(
        const JointNameToValueMap& joint_external_torques, 
        const JointNameToValueMap& joint_dynamical_torques) const;
    Eigen::Vector3d getExternalForceByNode(const std::string& node_name, 
        const JointNameToValueMap& joint_positions, 
        const JointNameToValueMap& external_torques) const;
    std::vector<Eigen::Vector3d> getWorldTorqueInLegs(
        const JointNameToValueMap& joint_positions,
        const JointNameToValueMap& joint_torques) const;
    std::vector<std::string> getJointNames() const;
    std::vector<Eigen::Vector3d> convertTorqueToForce(
        const JointNameToValueMap& joint_positions,
        const JointNameToValueMap& joint_torques) const;

    inline void setInertialOrientation(const Eigen::Quaterniond& orientation) { m_world_to_backpack_orientation = orientation; }

private:
    Eigen::VectorXd convertJointNameToValueMapToEigenVector(const JointNameToValueMap& joint_values) const;
    JointNameToValueMap recursiveNewtonEuler(
        const JointNameToValueMap& joint_positions,
        const JointNameToValueMap& joint_velocities,
        const JointNameToValueMap& joint_accelerations) const;
    std::unordered_map<std::string, Eigen::Vector3d> vectorizeJointTorqueVectors(const JointNameToValueMap& joint_torques) const;
    std::unordered_map<std::string, Eigen::Vector3d> orientateTorqueVectorsToWorld(
        const JointNameToValueMap& joint_positions,
        const std::unordered_map<std::string, Eigen::Vector3d>& joint_torque_vectors) const;
    std::unordered_map<std::string, Eigen::Vector3d> orientateTorqueVectorsToFoot(
        const JointNameToValueMap& joint_positions,
        const std::unordered_map<std::string, Eigen::Vector3d>& joint_torque_vectors) const;

    Eigen::Quaterniond m_world_to_backpack_orientation;

    KDL::Chain m_kdl_chain_leg_left;
    KDL::Chain m_kdl_chain_leg_right;
    const Eigen::Vector3d m_world_gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
};

#endif // MARCH_STATE_ESTIMATOR__TORQUE_CONVERTER_HPP_