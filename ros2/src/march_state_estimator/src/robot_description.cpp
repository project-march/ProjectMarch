#include "march_state_estimator/robot_description.hpp"

#include "rclcpp/rclcpp.hpp"
#include "march_state_estimator/robot_joint.hpp"
#include "march_state_estimator/robot_mass.hpp"

#include "ginac/ginac.h"
#include <cstring>

RobotDescription::~RobotDescription()
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Destructing RobotDescription...");
    for (auto & robot_node : m_robot_nodes)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Deleting robot_node %s ...", robot_node->getName().c_str());
        delete robot_node;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Destructing RobotDescription done");
}

void RobotDescription::parseURDF(const std::string & urdf_path)
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescription::parse");
    m_urdf_model.initFile(urdf_path);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescription::parse done");

    for (auto & link : m_urdf_model.links_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "link: %s", link.first.c_str());
        if (link.second->inertial == nullptr)
        {
            // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "link has no inertial");
            continue;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Link origin: %f %f %f", link.second->inertial->origin.position.x, link.second->inertial->origin.position.y, link.second->inertial->origin.position.z);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Link origin: %f %f %f %f", link.second->inertial->origin.rotation.x, link.second->inertial->origin.rotation.y, link.second->inertial->origin.rotation.z, link.second->inertial->origin.rotation.w);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "")
        RobotMass * robot_mass = new RobotMass(link.first, m_robot_nodes.size(), link.second->inertial->mass);
        
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
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "joint: %s", joint.first.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Joint origin: %f %f %f", joint.second->parent_to_joint_origin_transform.position.x, joint.second->parent_to_joint_origin_transform.position.y, joint.second->parent_to_joint_origin_transform.position.z);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Joint origin: %f %f %f %f", joint.second->parent_to_joint_origin_transform.rotation.x, joint.second->parent_to_joint_origin_transform.rotation.y, joint.second->parent_to_joint_origin_transform.rotation.z, joint.second->parent_to_joint_origin_transform.rotation.w);
        if (joint.second->type == JOINT_TYPE_REVOLUTE)
        {
            // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "joint is revolute or continuous");
            std::vector<double> joint_axis = {joint.second->axis.x, joint.second->axis.y, joint.second->axis.z};
            RobotJoint * robot_joint = new RobotJoint(joint.first, m_robot_nodes.size(), joint_axis);
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

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Size of robot_links_: %d", m_robot_nodes.size());
}

void RobotDescription::configureRobotNodes()
{
    // for (auto & robot_node : m_robot_nodes)
    // {
    //     std::string name = robot_node->getName();
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "robot_node: %s", name.c_str());
    //     if (robot_node->getParent() != nullptr)
    //     {
    //         RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Parent: %s", robot_node->getParent()->getName().c_str());
    //     }
    //     for (auto & child : robot_node->getChildren())
    //     {
    //         RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Child: %s", child->getName().c_str());
    //     }
    //     Eigen::Vector3d origin_position = robot_node->getOriginPosition();
    //     Eigen::Matrix3d origin_rotation = robot_node->getOriginRotation();
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Origin position: %f %f %f", origin_position.x(), origin_position.y(), origin_position.z());
    //     for (int i = 0; i < origin_rotation.rows(); i++)
    //     {
    //         RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Origin rotation: %f %f %f", origin_rotation(i, 0), origin_rotation(i, 1), origin_rotation(i, 2));
    //     }
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "---------------------");
    // }

    for (auto & robot_node : m_robot_nodes)
    {
        robot_node->expressKinematics();
    }
}

std::vector<std::string> RobotDescription::getNodeNames()
{
    std::vector<std::string> node_names;
    for (auto & robot_node : m_robot_nodes)
    {
        node_names.push_back(robot_node->getName());
    }
    return node_names;
}

std::vector<std::string> RobotDescription::getParentNames()
{
    std::vector<std::string> parent_names;
    for (auto & robot_node : m_robot_nodes)
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

std::vector<RobotNode*> RobotDescription::findNodes(std::vector<std::string> names)
{
    /*
     * Return a list of pointers to robot nodes given a list of specified names.
     * This is a utility function for Task Service.
     */
    std::vector<RobotNode*> robot_nodes;

    // TODO: Find std library for searching nodes given the names
    // TODO: Find mapping of name to vector
    for (auto & name : names)
    {
        for (auto & robot_node : m_robot_nodes)
        {
            if (robot_node->getName() != name)
            {
                continue;
            }
            
            robot_nodes.push_back(robot_node);
            break;
        }
    }

    return robot_nodes;
}

std::vector<Eigen::Vector3d> RobotDescription::getNodesPosition(std::vector<std::string> joint_names, std::vector<double> joint_angles)
{
    std::vector<Eigen::Vector3d> nodes_position;
    for (auto & robot_node : m_robot_nodes)
    {
        // std::vector<std::string> joint_names = {
        //     "left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", 
        //     "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};
        // std::vector<double> joint_angles = {
        //     0.0, 0.0, 0.0, 0.0, 
        //     0.0, 0.0, 0.0, 0.0};
        nodes_position.push_back(robot_node->getGlobalPosition(joint_names, joint_angles));
    }
    return nodes_position;
}

std::vector<Eigen::Matrix3d> RobotDescription::getNodesRotation(std::vector<std::string> joint_names, std::vector<double> joint_angles)
{
    std::vector<Eigen::Matrix3d> nodes_rotation;
    for (auto & robot_node : m_robot_nodes)
    {
        nodes_rotation.push_back(robot_node->getGlobalRotation(joint_names, joint_angles));
    }
    return nodes_rotation;
}