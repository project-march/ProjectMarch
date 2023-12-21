#include "march_state_estimator/robot_description.hpp"

#include "rclcpp/rclcpp.hpp"
#include "march_state_estimator/robot_joint.hpp"
#include "march_state_estimator/robot_mass.hpp"

#include "ginac/ginac.h"
#include <cstring>

RobotDescription::~RobotDescription()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destructing RobotDescription...");
    for (auto & robot_node : robot_nodes_)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Deleting robot_node %s ...", robot_node->getName().c_str());
        delete robot_node;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destructing RobotDescription done");
}

void RobotDescription::parseURDF(const std::string & urdf_path)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescription::parse");
    urdf_model_.initFile(urdf_path);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescription::parse done");

    for (auto & link : urdf_model_.links_)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "link: %s", link.first.c_str());
        if (link.second->inertial == nullptr)
        {
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "link has no inertial");
            continue;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link origin: %f %f %f", link.second->inertial->origin.position.x, link.second->inertial->origin.position.y, link.second->inertial->origin.position.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link origin: %f %f %f %f", link.second->inertial->origin.rotation.x, link.second->inertial->origin.rotation.y, link.second->inertial->origin.rotation.z, link.second->inertial->origin.rotation.w);
        RobotMass * robot_mass = new RobotMass(link.first, robot_nodes_.size(), link.second->inertial->mass);
        
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

        robot_nodes_.push_back(robot_mass);
    }

    for (auto & joint : urdf_model_.joints_)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "joint: %s", joint.first.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint origin: %f %f %f", joint.second->parent_to_joint_origin_transform.position.x, joint.second->parent_to_joint_origin_transform.position.y, joint.second->parent_to_joint_origin_transform.position.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint origin: %f %f %f %f", joint.second->parent_to_joint_origin_transform.rotation.x, joint.second->parent_to_joint_origin_transform.rotation.y, joint.second->parent_to_joint_origin_transform.rotation.z, joint.second->parent_to_joint_origin_transform.rotation.w);
        if (joint.second->type == JOINT_TYPE_REVOLUTE || joint.second->type == JOINT_TYPE_CONTINUOUS)
        {
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "joint is revolute or continuous");
            std::vector<double> joint_axis = {joint.second->axis.x, joint.second->axis.y, joint.second->axis.z};
            RobotJoint * robot_joint = new RobotJoint(joint.first, robot_nodes_.size(), joint_axis);
            for (auto & robot_node : robot_nodes_)
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

            robot_nodes_.push_back(robot_joint);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Size of robot_links_: %d", robot_nodes_.size());

    // GiNaC::symbol x("x"), y("y");
    // GiNaC::ex ex = x + y;

    // GiNaC::ex ex2 = GiNaC::diff(ex, x);
    // GiNaC::ex sol = GiNaC::evalf(ex2.subs(GiNaC::lst{x == 1.0, y == 5.0}));

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ex: %f", 
    //     GiNaC::ex_to<GiNaC::numeric>(sol).to_double());
}

void RobotDescription::configureRobotNodes()
{
    for (auto & robot_node : robot_nodes_)
    {
        std::string name = robot_node->getName();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robot_node: %s", name.c_str());
        if (robot_node->getParent() != nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parent: %s", robot_node->getParent()->getName().c_str());
        }
        for (auto & child : robot_node->getChildren())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Child: %s", child->getName().c_str());
        }
        Eigen::Vector3d origin_position = robot_node->getOriginPosition();
        Eigen::Matrix3d origin_rotation = robot_node->getOriginRotation();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Origin position: %f %f %f", origin_position.x(), origin_position.y(), origin_position.z());
        for (int i = 0; i < origin_rotation.rows(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Origin rotation: %f %f %f", origin_rotation(i, 0), origin_rotation(i, 1), origin_rotation(i, 2));
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------");
    }
}

std::vector<std::string> RobotDescription::getNodeNames()
{
    std::vector<std::string> node_names;
    for (auto & robot_node : robot_nodes_)
    {
        node_names.push_back(robot_node->getName());
    }
    return node_names;
}

std::vector<std::string> RobotDescription::getParentNames()
{
    std::vector<std::string> parent_names;
    for (auto & robot_node : robot_nodes_)
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

std::vector<Eigen::Vector3d> RobotDescription::getNodesPosition()
{
    std::vector<Eigen::Vector3d> nodes_position;
    for (auto & robot_node : robot_nodes_)
    {
        nodes_position.push_back(robot_node->getGlobalPosition());
    }
    return nodes_position;
}

std::vector<Eigen::Matrix3d> RobotDescription::getNodesRotation()
{
    std::vector<Eigen::Matrix3d> nodes_rotation;
    for (auto & robot_node : robot_nodes_)
    {
        nodes_rotation.push_back(robot_node->getGlobalRotation());
    }
    return nodes_rotation;
}