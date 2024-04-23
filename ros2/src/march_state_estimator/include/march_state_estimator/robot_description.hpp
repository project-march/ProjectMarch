/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_
#define MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "march_state_estimator/robot_node.hpp"
#include "march_state_estimator/robot_joint.hpp"
#include "yaml-cpp/yaml.h"

struct RobotPartData {
    std::string name;
    std::string type;
    double mass;
    std::string inertia;
    std::vector<double> joint_axis;
    std::vector<std::string> global_rotation;
    std::vector<std::string> global_linear_position;
    std::vector<std::string> global_linear_velocity;
    std::vector<std::string> global_linear_acceleration;
    std::vector<std::string> global_linear_position_jacobian;
    std::vector<std::string> global_rotation_jacobian;
    std::vector<std::string> dynamical_torque;
};

class RobotDescription {
public:
    typedef std::shared_ptr<RobotDescription> SharedPtr;

    RobotDescription(std::string yaml_filename);
    ~RobotDescription() = default;

    RobotNode::SharedPtr findNode(const std::string& name);
    std::vector<RobotNode::SharedPtr> findNodes(std::vector<std::string> names);
    inline std::vector<RobotJoint::SharedPtr> getJointNodes() const { return m_joint_node_ptrs; }

    std::vector<std::string> getAllNodeNames() const;
    std::vector<std::string> getAllParentNames() const;
    std::vector<Eigen::Vector3d> getAllNodesPosition(const std::unordered_map<std::string, double>& joint_positions);
    std::vector<Eigen::Matrix3d> getAllNodesRotation(const std::unordered_map<std::string, double>& joint_positions);
    Eigen::Quaterniond getInertialOrientation() const;

    void setInertialOrientation(const Eigen::Quaterniond& inertial_orientation);
    void setStanceLeg(const uint8_t& stance_leg, const Eigen::Vector3d& left_foot_position,
        const Eigen::Vector3d& right_foot_position);

private:
    void parseYAML(const std::string& yaml_path);
    void createRobotPart(const RobotPartData& robot_part_data);
    void setRobotPart(const std::shared_ptr<RobotNode> robot_node, const RobotPartData& robot_part_data);
    std::vector<std::string> vectorizeExpressions(
        const YAML::Node& yaml_node, const unsigned int& rows, const unsigned int& cols);

    std::vector<RobotNode::SharedPtr> m_robot_node_ptrs;
    std::vector<RobotJoint::SharedPtr> m_joint_node_ptrs;
    std::unordered_map<std::string, RobotNode::SharedPtr> m_robot_nodes_map;
    Eigen::Quaterniond m_inertial_orientation;
};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_