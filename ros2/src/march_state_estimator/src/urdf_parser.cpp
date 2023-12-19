#include "march_state_estimator/urdf_parser.hpp"

UrdfParser::UrdfParser()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UrdfParser constructor");
}

void UrdfParser::parse(const std::string & urdf_path)
{
    if (!urdf_model_.initFile(urdf_path))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not parse URDF file: %s", urdf_path.c_str());
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully parsed URDF file: %s", urdf_path.c_str());

    // Access URDF model data
    for (const auto & joint : urdf_model_.joints_)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint name: %s", joint.first.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint parent link name: %s", joint.second->parent_link_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint child link name: %s", joint.second->child_link_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint type: %d", joint.second->type);
        // Type 1 = Revolute
        // Type 2 = Continuous
        // Type 3 = Prismatic
        // Type 4 = Floating
        // Type 5 = Planar
        // Type 6 = Fixed
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint axis: %f %f %f", joint.second->axis.x, joint.second->axis.y, joint.second->axis.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint origin: %f %f %f", joint.second->parent_to_joint_origin_transform.position.x, joint.second->parent_to_joint_origin_transform.position.y, joint.second->parent_to_joint_origin_transform.position.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint origin: %f %f %f %f", joint.second->parent_to_joint_origin_transform.rotation.x, joint.second->parent_to_joint_origin_transform.rotation.y, joint.second->parent_to_joint_origin_transform.rotation.z, joint.second->parent_to_joint_origin_transform.rotation.w);
        if (joint.second->type == 1 || joint.second->type == 2)
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint limits: %f %f", joint.second->limits->lower, joint.second->limits->upper);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------");
    }

    for (const auto & link : urdf_model_.links_)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link name: %s", link.first.c_str());
        if (link.second->inertial == nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link has no inertial");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------");
            continue;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link inertial mass: %f", link.second->inertial->mass);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link inertial origin: %f %f %f", link.second->inertial->origin.position.x, link.second->inertial->origin.position.y, link.second->inertial->origin.position.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link inertial origin: %f %f %f %f", link.second->inertial->origin.rotation.x, link.second->inertial->origin.rotation.y, link.second->inertial->origin.rotation.z, link.second->inertial->origin.rotation.w);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Link inertial inertia: %f %f %f %f %f %f", link.second->inertial->ixx, link.second->inertial->ixy, link.second->inertial->ixz, link.second->inertial->iyy, link.second->inertial->iyz, link.second->inertial->izz);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------");
    }
}