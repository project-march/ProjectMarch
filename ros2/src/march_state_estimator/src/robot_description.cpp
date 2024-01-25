/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description.hpp"

#include "rclcpp/rclcpp.hpp"
#include "march_state_estimator/robot_joint.hpp"
#include "march_state_estimator/robot_mass.hpp"

#include "boost/range/combine.hpp"
#include "ginac/ginac.h"
#include <cstring>
#include <unordered_map>
#include <algorithm>

#include "ament_index_cpp/get_package_share_directory.hpp"

void RobotDescription::parseYAML(const std::string & yaml_path)
{
    RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescription::parseYAML");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("march_state_estimator");
    YAML::Node yaml_node = YAML::LoadFile(package_share_directory + "/config/" + yaml_path);

    const std::vector<std::string> part_names = yaml_node["names"].as<std::vector<std::string>>();
    std::vector<RobotPartData> robot_part_datas;
    RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "Number of parts: %d", part_names.size());

    for (auto & name : part_names)
    {
        RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "Part name: %s", name.c_str());
        RobotPartData robot_part_data;
        robot_part_data.name = name;
        robot_part_data.type = yaml_node[name]["type"].as<std::string>();
        if (robot_part_data.type == "joint")
        {
            robot_part_data.joint_axis = yaml_node[name]["axis"].as<std::vector<double>>();
        }
        unsigned int DOF = yaml_node[name]["joint_names"].as<std::vector<std::string>>().size();

        robot_part_data.global_rotation = vectorizeExpressions(yaml_node[name]["rotation"], WORKSPACE_DIM, WORKSPACE_DIM);
        robot_part_data.global_linear_position = vectorizeExpressions(yaml_node[name]["linear"]["position"], WORKSPACE_DIM, 1);
        robot_part_data.global_linear_position_jacobian = vectorizeExpressions(yaml_node[name]["jacobian"]["linear_position"], WORKSPACE_DIM, DOF);
        robot_part_data.global_rotation_jacobian = vectorizeExpressions(yaml_node[name]["jacobian"]["rotation"], WORKSPACE_DIM, DOF);

        createRobotPart(robot_part_data);
        robot_part_datas.push_back(robot_part_data);
    }

    // Implement hash map given its name
    for (auto robot_node : m_robot_node_ptrs)
    {
        if (robot_node->getName() == "backpack")
        {
            robot_node->setParent(nullptr);
            continue;
        }

        std::string parent_name = yaml_node[robot_node->getName()]["parent"].as<std::string>();
        for (auto & robot_node_parent : m_robot_node_ptrs)
        {
            if (robot_node_parent->getName() == parent_name)
            {
                robot_node_parent->addChild(robot_node);
                break;
            }
        }

        std::vector<std::shared_ptr<RobotNode>> joint_nodes = findNodes(yaml_node[robot_node->getName()]["joint_names"].as<std::vector<std::string>>());
        robot_node->setJointNodes(joint_nodes);
    }

    for (auto tuple : boost::combine(m_robot_node_ptrs, robot_part_datas))
    {
        std::shared_ptr<RobotNode> robot_node;
        RobotPartData robot_data;
        boost::tie(robot_node, robot_data) = tuple;

        setRobotPart(robot_node, robot_data);
    }

    RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescription::parseYAML done");
}

void RobotDescription::createRobotPart(const RobotPartData & robot_part_data)
{
    if (robot_part_data.type == "mass")
    {
        RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescription::createRobotPart: Creating mass");
        std::shared_ptr<RobotMass> robot_mass = std::make_shared<RobotMass>(robot_part_data.name, m_robot_node_ptrs.size(), 0.0);

        // robot_mass->setExpressionGlobalRotation(robot_part_data.global_rotation);
        // robot_mass->setExpressionGlobalPosition(robot_part_data.global_linear_position);
        // robot_mass->setExpressionGlobalPositionJacobian(robot_part_data.global_linear_position_jacobian);
        // robot_mass->setExpressionGlobalRotationJacobian(robot_part_data.global_rotation_jacobian);

        m_robot_node_ptrs.push_back(robot_mass);
        m_robot_nodes_map[robot_part_data.name] = robot_mass;
    }
    else if (robot_part_data.type == "joint")
    {
        RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescription::createRobotPart: Creating joint");
        std::shared_ptr<RobotJoint> robot_joint = std::make_shared<RobotJoint>(robot_part_data.name, m_robot_node_ptrs.size(), robot_part_data.joint_axis);

        // robot_joint->setExpressionGlobalRotation(robot_part_data.global_rotation);
        // robot_joint->setExpressionGlobalPosition(robot_part_data.global_linear_position);
        // robot_joint->setExpressionGlobalPositionJacobian(robot_part_data.global_linear_position_jacobian);
        // robot_joint->setExpressionGlobalRotationJacobian(robot_part_data.global_rotation_jacobian);

        m_robot_node_ptrs.push_back(robot_joint);
        m_robot_nodes_map[robot_part_data.name] = robot_joint;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"), "RobotDescription::createRobotPart: Unknown robot part type");
    }
}

void RobotDescription::setRobotPart(const std::shared_ptr<RobotNode> robot_node, const RobotPartData & robot_part_data)
{
    robot_node->setExpressionGlobalPosition(robot_part_data.global_linear_position);
    robot_node->setExpressionGlobalRotation(robot_part_data.global_rotation);
    robot_node->setExpressionGlobalPositionJacobian(robot_part_data.global_linear_position_jacobian);
    robot_node->setExpressionGlobalRotationJacobian(robot_part_data.global_rotation_jacobian);
}

void RobotDescription::parseURDF(const std::string & urdf_path)
{
    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescription::parse");
    m_urdf_model.initFile(urdf_path);
    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescription::parse done");

    for (auto & link : m_urdf_model.links_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "link: %s", link.first.c_str());
        if (link.second->inertial == nullptr)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "link has no inertial");
            continue;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "Link origin: %f %f %f", link.second->inertial->origin.position.x, link.second->inertial->origin.position.y, link.second->inertial->origin.position.z);
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "Link origin: %f %f %f %f", link.second->inertial->origin.rotation.x, link.second->inertial->origin.rotation.y, link.second->inertial->origin.rotation.z, link.second->inertial->origin.rotation.w);
        std::shared_ptr<RobotMass> robot_mass = std::make_shared<RobotMass>(link.first, m_robot_nodes.size(), link.second->inertial->mass);
        
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Matrix3d rotation;
        if (robot_mass->getName() != "backpack")
        {
            position = {link.second->inertial->origin.position.x, link.second->inertial->origin.position.y, link.second->inertial->origin.position.z};
            orientation = {link.second->inertial->origin.rotation.w, link.second->inertial->origin.rotation.x, link.second->inertial->origin.rotation.y, link.second->inertial->origin.rotation.z};
        }
        else
        {
            position = {0.0, 0.0, 0.0};
            orientation = {1.0, 0.0, 0.0, 0.0};
        }
        rotation = orientation.toRotationMatrix();
        robot_mass->setOriginPosition(position);
        robot_mass->setOriginRotation(rotation);

        m_robot_nodes.push_back(robot_mass);
    }

    for (auto & joint : m_urdf_model.joints_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "joint: %s", joint.first.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "Joint origin: %f %f %f", joint.second->parent_to_joint_origin_transform.position.x, joint.second->parent_to_joint_origin_transform.position.y, joint.second->parent_to_joint_origin_transform.position.z);
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "Joint origin: %f %f %f %f", joint.second->parent_to_joint_origin_transform.rotation.x, joint.second->parent_to_joint_origin_transform.rotation.y, joint.second->parent_to_joint_origin_transform.rotation.z, joint.second->parent_to_joint_origin_transform.rotation.w);
        if (joint.second->type == JOINT_TYPE_REVOLUTE)
        {
            std::vector<double> joint_axis = {joint.second->axis.x, joint.second->axis.y, joint.second->axis.z};
            std::shared_ptr<RobotJoint> robot_joint = std::make_shared<RobotJoint>(joint.first, m_robot_nodes.size(), joint_axis);
            for (auto & robot_node : m_robot_nodes)
            {
                if (robot_node->getName() == joint.second->parent_link_name)
                {
                    robot_joint->setParent(robot_node);
                    robot_node->addChild(robot_joint);
                }
                if (robot_node->getName() == joint.second->child_link_name)
                {
                    robot_joint->addChild(robot_node);
                    robot_node->setParent(robot_joint);
                }
            }
            Eigen::Vector3d position = {joint.second->parent_to_joint_origin_transform.position.x, joint.second->parent_to_joint_origin_transform.position.y, joint.second->parent_to_joint_origin_transform.position.z};
            Eigen::Quaterniond orientation = {joint.second->parent_to_joint_origin_transform.rotation.w, joint.second->parent_to_joint_origin_transform.rotation.x, joint.second->parent_to_joint_origin_transform.rotation.y, joint.second->parent_to_joint_origin_transform.rotation.z};
            Eigen::Matrix3d rotation = orientation.toRotationMatrix();
            robot_joint->setOriginPosition(position);
            robot_joint->setOriginRotation(rotation);

            m_robot_nodes.push_back(robot_joint);
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "Size of robot_links_: %d", m_robot_nodes.size());
}

void RobotDescription::configureRobotNodes()
{
    for (auto & robot_node : m_robot_nodes)
    {
        robot_node->expressRotation();
    }

    for (auto & robot_node : m_robot_nodes)
    {
        robot_node->expressKinematics();
    }
}

std::vector<std::string> RobotDescription::getAllNodeNames() const
{
    std::vector<std::string> node_names;
    for (const auto & robot_node : m_robot_node_ptrs)
    {
        node_names.push_back(robot_node->getName());
    }
    return node_names;
}

std::vector<std::string> RobotDescription::getAllParentNames() const
{
    std::vector<std::string> parent_names;
    for (auto & robot_node : m_robot_node_ptrs)
    {
        if (robot_node->getParent() != nullptr)
        {
            parent_names.push_back(robot_node->getParent()->getName());
        }
        else
        {
            parent_names.push_back("none");
        }
    }
    return parent_names;
}

std::vector<std::shared_ptr<RobotNode>> RobotDescription::findNodes(std::vector<std::string> names)
{
    std::vector<std::shared_ptr<RobotNode>> robot_nodes;
    for (auto & name : names)
    {
        try
        {
            robot_nodes.push_back(m_robot_nodes_map.at(name));
        }
        catch (const std::out_of_range & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"), 
                "RobotDescription::findNodes: Cannot find node %s, error %s", name.c_str(), e.what());
        }
    }
    return robot_nodes;
}

std::vector<Eigen::Vector3d> RobotDescription::getAllNodesPosition(const std::unordered_map<std::string, double> & joint_positions)
{
    std::vector<Eigen::Vector3d> nodes_position;
    for (const auto & robot_node : m_robot_node_ptrs)
    {
        nodes_position.push_back(robot_node->getGlobalPosition(joint_positions));
    }
    return nodes_position;
}

std::vector<Eigen::Matrix3d> RobotDescription::getAllNodesRotation(const std::unordered_map<std::string, double> & joint_positions)
{
    std::vector<Eigen::Matrix3d> nodes_rotation;
    for (const auto & robot_node : m_robot_node_ptrs)
    {
        nodes_rotation.push_back(robot_node->getGlobalRotation(joint_positions));
    }
    return nodes_rotation;
}

std::vector<std::string> RobotDescription::vectorizeExpressions(
    const YAML::Node & yaml_node, const unsigned int & rows, const unsigned int & cols)
{
    std::vector<std::string> expressions;
    for (unsigned int i = 0; i < rows; i++)
    {
        for (unsigned int j = 0; j < cols; j++)
        {
            std::string expression = yaml_node["m" + std::to_string(i) + std::to_string(j)].as<std::string>();
            RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescription::vectorizeExpressions: %s", expression.c_str());
            expressions.push_back(expression);
        }
    }
    return expressions;
}