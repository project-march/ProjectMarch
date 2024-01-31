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
#include "urdf/model.h"
#include "yaml-cpp/yaml.h"

struct RobotPartData {
    std::string name;
    std::string type;
    double mass;
    std::string inertia;
    std::vector<double> joint_axis;
    std::vector<std::string> global_rotation;
    std::vector<std::string> global_linear_position;
    std::vector<std::string> global_linear_position_jacobian;
    std::vector<std::string> global_rotation_jacobian;
    std::vector<std::string> dynamical_torque;
};

class RobotDescription {
public:
    RobotDescription() = default;
    ~RobotDescription() = default;

    void parseYAML(const std::string& yaml_path);
    RobotNode::SharedPtr findNode(const std::string& name);
    std::vector<RobotNode::SharedPtr> findNodes(std::vector<std::string> names);

    std::vector<std::string> getAllNodeNames() const;
    std::vector<std::string> getAllParentNames() const;
    std::vector<Eigen::Vector3d> getAllNodesPosition(const std::unordered_map<std::string, double>& joint_positions);
    std::vector<Eigen::Matrix3d> getAllNodesRotation(const std::unordered_map<std::string, double>& joint_positions);

private:
    void createRobotPart(const RobotPartData& robot_part_data);
    void setRobotPart(const std::shared_ptr<RobotNode> robot_node, const RobotPartData& robot_part_data);
    std::string vectorizeExpression(const YAML::Node& yaml_node);
    std::vector<std::string> vectorizeExpressions(
        const YAML::Node& yaml_node, const unsigned int& rows, const unsigned int& cols);

    std::vector<RobotNode::SharedPtr> m_robot_node_ptrs; // TODO: Remove this and use m_robot_nodes_map instead.
    std::unordered_map<std::string, RobotNode::SharedPtr> m_robot_nodes_map;
};

#endif // MARCH_STATE_ESTIMATOR__ROBOT_DESCRIPTION_HPP_