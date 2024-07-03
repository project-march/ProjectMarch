/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description/robot_description.hpp"

#include "march_state_estimator/robot_description/robot_mass.hpp"
#include "march_state_estimator/robot_description/robot_zmp.hpp"
#include "rclcpp/rclcpp.hpp"

#include "boost/range/combine.hpp"
#include "ginac/ginac.h"
#include <algorithm>
#include <cstring>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"

RobotDescription::RobotDescription(std::string yaml_filename)
{
    m_robot_node_ptrs.clear();
    std::string yaml_filepath = ament_index_cpp::get_package_share_directory("march_state_estimator") + "/config/";
    parseYAML(yaml_filepath + yaml_filename);
}

void RobotDescription::parseYAML(const std::string& yaml_path)
{
    try {
        YAML::Node yaml_node = YAML::LoadFile(yaml_path);

        const std::vector<std::string> part_names = yaml_node["names"].as<std::vector<std::string>>();
        std::vector<RobotPartData> robot_part_datas;

        // Create data structures for robot parts and initialize the robot parts.
        // std::cout << "Create data structures for robot parts and initialize the robot parts." << std::endl;
        for (auto& name : part_names) {
            RobotPartData robot_part_data;
            robot_part_data.name = name;
            robot_part_data.type = yaml_node[name]["type"].as<std::string>();

            robot_part_data.mass = yaml_node[name]["dynamics"]["mass"].as<double>();
            robot_part_data.inertia = yaml_node[name]["dynamics"]["inertia"].as<std::string>();

            robot_part_data.joint_axis = yaml_node[name]["axis"].as<std::vector<double>>();
            const unsigned int ABSOLUTE_DOF
                = yaml_node[name]["joint_names"]["absolute"].as<std::vector<std::string>>().size();
            const unsigned int RELATIVE_DOF
                = yaml_node[name]["joint_names"]["relative"].as<std::vector<std::string>>().size();

            robot_part_data.global_rotation
                = vectorizeExpressions(yaml_node[name]["rotation"], WORKSPACE_DIM, WORKSPACE_DIM);
            robot_part_data.global_linear_position
                = vectorizeExpressions(yaml_node[name]["linear"]["position"], WORKSPACE_DIM, 1);
            robot_part_data.global_linear_velocity
                = vectorizeExpressions(yaml_node[name]["linear"]["velocity"], WORKSPACE_DIM, 1);
            robot_part_data.global_linear_acceleration
                = vectorizeExpressions(yaml_node[name]["linear"]["acceleration"], WORKSPACE_DIM, 1);
            robot_part_data.global_linear_position_jacobian
                = vectorizeExpressions(yaml_node[name]["jacobian"]["position"], WORKSPACE_DIM, ABSOLUTE_DOF);
            robot_part_data.global_rotation_jacobian
                = vectorizeExpressions(yaml_node[name]["jacobian"]["rotation"], WORKSPACE_DIM, ABSOLUTE_DOF);
            robot_part_data.dynamical_torque = vectorizeExpressions(yaml_node[name]["dynamics"]["torque"], RELATIVE_DOF, 1);

            createRobotPart(robot_part_data);
            robot_part_datas.push_back(robot_part_data);
        }

        // Set the parent and children of each robot node, including the absolute and relative joints.
        // std::cout << "Set the parent and children of each robot node, including the absolute and relative joints." << std::endl;
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

        // Set the expressions for each robot node.
        // std::cout << "Set the expressions for each robot node." << std::endl;
        for (auto tuple : boost::combine(m_robot_node_ptrs, robot_part_datas)) {
            RobotNode::SharedPtr robot_node;
            RobotPartData robot_data;
            boost::tie(robot_node, robot_data) = tuple;
            setRobotPart(robot_node, robot_data);
        }
    } catch (const YAML::Exception& e) {
        // TODO: Store logger in a member variable
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"),
            "RobotDescription::parseYAML: Cannot parse YAML file %s, error %s", yaml_path.c_str(), e.what());
    }
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

void RobotDescription::createRobotPart(const RobotPartData& robot_part_data)
{
    if (robot_part_data.type == "zmp") {
        std::shared_ptr<RobotZMP> robot_zmp = std::make_shared<RobotZMP>();
        m_robot_node_ptrs.push_back(robot_zmp);
        m_robot_nodes_map[robot_part_data.name] = robot_zmp;
    } else if (robot_part_data.type == "mass" || robot_part_data.type == "com") {
        std::shared_ptr<RobotMass> robot_mass
            = std::make_shared<RobotMass>(robot_part_data.name, m_robot_node_ptrs.size(), robot_part_data.mass);
        m_robot_node_ptrs.push_back(robot_mass);
        m_robot_nodes_map[robot_part_data.name] = robot_mass;
    } else if (robot_part_data.type == "joint") {
        std::shared_ptr<RobotJoint> robot_joint
            = std::make_shared<RobotJoint>(robot_part_data.name, m_robot_node_ptrs.size(), robot_part_data.joint_axis);
        m_robot_node_ptrs.push_back(robot_joint);
        m_joint_node_ptrs.push_back(std::dynamic_pointer_cast<RobotJoint>(robot_joint));
        m_robot_nodes_map[robot_part_data.name] = robot_joint;
    } else {
        RCLCPP_ERROR(
            rclcpp::get_logger("state_estimator_node"), "RobotDescription::createRobotPart: Unknown robot part type");
    }
}

Eigen::Quaterniond RobotDescription::getInertialOrientation() const
{
    return m_inertial_orientation;
}

void RobotDescription::setInertialOrientation(const Eigen::Quaterniond& inertial_orientation)
{
    m_inertial_orientation = inertial_orientation;

    // Set the orientation of abstract robot parts.
    std::dynamic_pointer_cast<RobotZMP>(m_robot_nodes_map.at("zmp"))->setInertialOrientation(inertial_orientation);
}

void RobotDescription::setStanceLeg(
    const uint8_t& stance_leg, const Eigen::Vector3d& left_foot_position, const Eigen::Vector3d& right_foot_position)
{
    std::dynamic_pointer_cast<RobotZMP>(m_robot_nodes_map.at("zmp"))
        ->setFootPositions(left_foot_position, right_foot_position);
    std::dynamic_pointer_cast<RobotZMP>(m_robot_nodes_map.at("zmp"))->setStanceLeg(stance_leg);
}

void RobotDescription::setRobotPart(const RobotNode::SharedPtr robot_node, const RobotPartData& robot_part_data)
{
    robot_node->setExpressionRelativeInertia(robot_part_data.inertia);
    robot_node->setExpressionGlobalPosition(robot_part_data.global_linear_position);
    robot_node->setExpressionGlobalVelocity(robot_part_data.global_linear_velocity);
    robot_node->setExpressionGlobalAcceleration(robot_part_data.global_linear_acceleration);
    robot_node->setExpressionGlobalRotation(robot_part_data.global_rotation);
    robot_node->setExpressionGlobalPositionJacobian(robot_part_data.global_linear_position_jacobian);
    robot_node->setExpressionGlobalRotationJacobian(robot_part_data.global_rotation_jacobian);
    robot_node->setExpressionDynamicalTorque(robot_part_data.dynamical_torque);
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