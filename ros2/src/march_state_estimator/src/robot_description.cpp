#include "march_state_estimator/robot_description.hpp"

#include "rclcpp/rclcpp.hpp"
#include "march_state_estimator/robot_joint.hpp"
#include "march_state_estimator/robot_mass.hpp"

// #include "ginac/ginac.h"
#include <iostream>

RobotDescription::~RobotDescription()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Destructing RobotDescription...");
    // for (auto & robot_node : robot_nodes_)
    // {
    //     delete robot_node;
    // }
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "link has no inertial");
            continue;
        }
        RobotMass robot_mass = RobotMass(link.first, link.second->inertial->mass);
        robot_nodes_.push_back(&robot_mass);
    }

    for (auto & joint : urdf_model_.joints_)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "joint: %s", joint.first.c_str());
        if (joint.second->type == 1 || joint.second->type == 2)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "joint is revolute or continuous");
            std::vector<double> joint_axis = {joint.second->axis.x, joint.second->axis.y, joint.second->axis.z};
            RobotJoint robot_joint = RobotJoint(joint.first, joint_axis);
            robot_nodes_.push_back(&robot_joint);
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