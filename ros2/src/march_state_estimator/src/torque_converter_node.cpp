/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/torque_converter_node.hpp"

#include "omp.h"

TorqueConverterNode::TorqueConverterNode()
    : Node("torque_converter_node")
{
    declare_parameter("robot_definition", std::string());
    std::string yaml_filename = get_parameter("robot_definition").as_string();

    if (yaml_filename.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No robot description file has been provided.");
        return;
    }

    m_robot_description = std::make_shared<RobotDescription>(yaml_filename);

    declare_parameter("urdf_file_path", std::string());
    std::string urdf_file_path = get_parameter("urdf_file_path").as_string();

    if (urdf_file_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No URDF file path has been provided.");
        return;
    }

    m_torque_converter = std::make_unique<TorqueConverter>(m_robot_description, urdf_file_path);

    m_joint_names.clear();
    m_joint_positions.clear();
    m_joint_velocities.clear();
    m_joint_accelerations.clear();
    m_joint_external_torques.clear();
    for (const auto& joint_node : m_robot_description->getJointNodes())
    {
        m_joint_positions[joint_node->getName()] = 0.0;
        m_joint_velocities[joint_node->getName()] = 0.0;
        m_joint_accelerations[joint_node->getName()] = 0.0;
        m_joint_external_torques[joint_node->getName()] = 0.0;
    }

    m_state_estimation_sub = this->create_subscription<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10, std::bind(&TorqueConverterNode::stateEstimationCallback, this, std::placeholders::_1));
    m_desired_joint_trajectory_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10, std::bind(&TorqueConverterNode::desiredJointTrajectoryCallback, this, std::placeholders::_1));
    m_joint_efforts_pub = this->create_publisher<march_shared_msgs::msg::JointEfforts>("state_estimation/desired_torque", 10);

    RCLCPP_INFO(this->get_logger(), "TorqueConverterNode has been initialized.");
}

void TorqueConverterNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    m_joint_names = msg->joint_state.name;
    for (unsigned long int i = 0; i < msg->joint_state.name.size(); i++)
    {
        m_joint_accelerations[msg->joint_state.name[i]] = msg->dynamical_joint_state.joint_acceleration[i];
        m_joint_external_torques[msg->joint_state.name[i]] = msg->dynamical_joint_state.effort_external[i];
    }
    m_torque_converter->setInertialOrientation(Eigen::Quaterniond(msg->imu.orientation.w, msg->imu.orientation.x, msg->imu.orientation.y, msg->imu.orientation.z));
}

void TorqueConverterNode::desiredJointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    for (unsigned long int i = 0; i < msg->joint_names.size(); i++)
    {
        m_joint_positions[msg->joint_names[i]] = msg->points.back().positions[i];
        m_joint_velocities[msg->joint_names[i]] = msg->points.back().velocities[i];
    }

    publishDesiredJointEfforts();
}

void TorqueConverterNode::publishDesiredJointEfforts()
{
    march_shared_msgs::msg::JointEfforts joint_efforts_msg;
    joint_efforts_msg.header.stamp = this->now();
    joint_efforts_msg.header.frame_id = "joint_link";

    // Calculate the dynamical joint efforts according to the desired joint positions and velocities
    RobotNode::JointNameToValueMap dynamical_joint_torques
        = m_torque_converter->getDynamicalTorques(m_joint_positions, m_joint_velocities, m_joint_accelerations);
    RobotNode::JointNameToValueMap total_joint_efforts
        = m_torque_converter->getTotalTorques(dynamical_joint_torques, m_joint_external_torques);

    for (const auto& joint_name : m_joint_names) {
        joint_efforts_msg.total_effort.push_back(total_joint_efforts[joint_name]);
        joint_efforts_msg.dynamical_effort.push_back(dynamical_joint_torques[joint_name]);
        joint_efforts_msg.external_effort.push_back(m_joint_external_torques[joint_name]);
    }

    m_joint_efforts_pub->publish(joint_efforts_msg);
}

int main(int argc, char** argv)
{
    Eigen::initParallel();
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_torque_converter = std::make_shared<TorqueConverterNode>();
    rclcpp::spin(node_torque_converter);
    rclcpp::shutdown();
    return 0;
}