/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description.hpp"

#include "march_state_estimator/robot_joint.hpp"
#include "march_state_estimator/robot_mass.hpp"
#include "rclcpp/rclcpp.hpp"

#include "boost/range/combine.hpp"
#include "ginac/ginac.h"
#include <algorithm>
#include <cstring>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"

void RobotDescription::parseYAML(const std::string& yaml_path)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("march_state_estimator");
    YAML::Node yaml_node = YAML::LoadFile(package_share_directory + "/config/" + yaml_path);

    const std::vector<std::string> part_names = yaml_node["names"].as<std::vector<std::string>>();
    std::vector<RobotPartData> robot_part_datas;

    for (auto& name : part_names) {
        RobotPartData robot_part_data;
        robot_part_data.name = name;
        robot_part_data.type = yaml_node[name]["type"].as<std::string>();
        robot_part_data.mass = yaml_node[name]["dynamics"]["mass"].as<double>();
        if (robot_part_data.type == "joint") {
            robot_part_data.joint_axis = yaml_node[name]["axis"].as<std::vector<double>>();
        }
        const unsigned int ABSOLUTE_DOF
            = yaml_node[name]["joint_names"]["absolute"].as<std::vector<std::string>>().size();
        const unsigned int RELATIVE_DOF
            = yaml_node[name]["joint_names"]["relative"].as<std::vector<std::string>>().size();

        robot_part_data.inertia = vectorizeExpression(yaml_node[name]["dynamics"]["inertia"]);
        robot_part_data.global_rotation
            = vectorizeExpressions(yaml_node[name]["rotation"], WORKSPACE_DIM, WORKSPACE_DIM);
        robot_part_data.global_linear_position
            = vectorizeExpressions(yaml_node[name]["linear"]["position"], WORKSPACE_DIM, 1);
        robot_part_data.global_linear_position_jacobian
            = vectorizeExpressions(yaml_node[name]["jacobian"]["position"], WORKSPACE_DIM, ABSOLUTE_DOF);
        robot_part_data.global_rotation_jacobian
            = vectorizeExpressions(yaml_node[name]["jacobian"]["rotation"], WORKSPACE_DIM, ABSOLUTE_DOF);
        robot_part_data.dynamical_torque = vectorizeExpressions(yaml_node[name]["dynamics"]["torque"], RELATIVE_DOF, 1);

        createRobotPart(robot_part_data);
        robot_part_datas.push_back(robot_part_data);
    }

    for (const auto& robot_node : m_robot_node_ptrs) {
        std::vector<std::string> children_name
            = yaml_node[robot_node->getName()]["children"].as<std::vector<std::string>>();
        for (const auto child_name : children_name) {
            robot_node->addChild(findNode(child_name));
        }

        std::vector<RobotNode::SharedPtr> absolute_joint_nodes
            = findNodes(yaml_node[robot_node->getName()]["joint_names"]["absolute"].as<std::vector<std::string>>());
        std::vector<RobotNode::SharedPtr> relative_joint_nodes
            = findNodes(yaml_node[robot_node->getName()]["joint_names"]["relative"].as<std::vector<std::string>>());
        robot_node->setJointNodes(absolute_joint_nodes, relative_joint_nodes);
    }

    for (auto tuple : boost::combine(m_robot_node_ptrs, robot_part_datas)) {
        RobotNode::SharedPtr robot_node;
        RobotPartData robot_data;
        boost::tie(robot_node, robot_data) = tuple;
        setRobotPart(robot_node, robot_data);
    }
}

void RobotDescription::createRobotPart(const RobotPartData& robot_part_data)
{
    if (robot_part_data.type == "mass") {
        std::shared_ptr<RobotMass> robot_mass
            = std::make_shared<RobotMass>(robot_part_data.name, m_robot_node_ptrs.size(), robot_part_data.mass);
        m_robot_node_ptrs.push_back(robot_mass);
        m_robot_nodes_map[robot_part_data.name] = robot_mass;
    } else if (robot_part_data.type == "joint") {
        std::shared_ptr<RobotJoint> robot_joint
            = std::make_shared<RobotJoint>(robot_part_data.name, m_robot_node_ptrs.size(), robot_part_data.joint_axis);
        m_robot_node_ptrs.push_back(robot_joint);
        m_robot_nodes_map[robot_part_data.name] = robot_joint;
    } else {
        RCLCPP_ERROR(
            rclcpp::get_logger("state_estimator_node"), "RobotDescription::createRobotPart: Unknown robot part type");
    }
}

void RobotDescription::setRobotPart(const RobotNode::SharedPtr robot_node, const RobotPartData& robot_part_data)
{
    robot_node->setExpressionRelativeInertia(robot_part_data.inertia);
    robot_node->setExpressionGlobalPosition(robot_part_data.global_linear_position);
    robot_node->setExpressionGlobalRotation(robot_part_data.global_rotation);
    robot_node->setExpressionGlobalPositionJacobian(robot_part_data.global_linear_position_jacobian);
    robot_node->setExpressionGlobalRotationJacobian(robot_part_data.global_rotation_jacobian);
    robot_node->setExpressionDynamicalTorque(robot_part_data.dynamical_torque);
}

std::vector<std::string> RobotDescription::getAllNodeNames() const
{
    std::vector<std::string> node_names;
    for (const auto& robot_node : m_robot_node_ptrs) {
        node_names.push_back(robot_node->getName());
    }
    return node_names;
}

std::vector<std::string> RobotDescription::getAllParentNames() const
{
    std::vector<std::string> parent_names;
    for (auto& robot_node : m_robot_node_ptrs) {
        if (robot_node->getParent() != nullptr) {
            parent_names.push_back(robot_node->getParent()->getName());
        } else {
            parent_names.push_back("none");
        }
    }
    return parent_names;
}

RobotNode::SharedPtr RobotDescription::findNode(const std::string& name)
{
    try {
        return m_robot_nodes_map.at(name);
    } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"),
            "RobotDescription::findNode: Cannot find node %s, error %s", name.c_str(), e.what());
        return nullptr;
    }
}

std::vector<RobotNode::SharedPtr> RobotDescription::findNodes(std::vector<std::string> names)
{
    std::vector<RobotNode::SharedPtr> robot_nodes;
    for (auto& name : names) {
        robot_nodes.push_back(findNode(name));
    }
    return robot_nodes;
}

std::vector<Eigen::Vector3d> RobotDescription::getAllNodesPosition(
    const std::unordered_map<std::string, double>& joint_positions)
{
    std::vector<Eigen::Vector3d> nodes_position;
    for (const auto& robot_node : m_robot_node_ptrs) {
        nodes_position.push_back(robot_node->getGlobalPosition(joint_positions));
    }
    return nodes_position;
}

std::vector<Eigen::Matrix3d> RobotDescription::getAllNodesRotation(
    const std::unordered_map<std::string, double>& joint_positions)
{
    std::vector<Eigen::Matrix3d> nodes_rotation;
    for (const auto& robot_node : m_robot_node_ptrs) {
        nodes_rotation.push_back(robot_node->getGlobalRotation(joint_positions));
    }
    return nodes_rotation;
}

std::string RobotDescription::vectorizeExpression(const YAML::Node& yaml_node)
{
    return yaml_node.as<std::string>();
}

std::vector<std::string> RobotDescription::vectorizeExpressions(
    const YAML::Node& yaml_node, const unsigned int& rows, const unsigned int& cols)
{
    std::vector<std::string> expressions;
    for (unsigned int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < cols; j++) {
            std::string expression = yaml_node["m" + std::to_string(i) + std::to_string(j)].as<std::string>();
            expressions.push_back(expression);
        }
    }
    return expressions;
}