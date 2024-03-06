/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_state_estimator/robot_description_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "march_shared_msgs/msg/node_jacobian.hpp"
#include <chrono>
#include <functional>
#include <unordered_map>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "yaml-cpp/yaml.h"

RobotDescriptionNode::RobotDescriptionNode(std::shared_ptr<RobotDescription> robot_description)
    : Node("robot_description_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Robot Description Node");

    m_robot_description = robot_description;

    m_service_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // m_subscription_state_estimation
    //     = this->create_subscription<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10,
    //         std::bind(&RobotDescriptionNode::stateEstimationCallback, this, std::placeholders::_1));
    // m_publisher_state_estimator_visualization
    //     = this->create_publisher<march_shared_msgs::msg::StateEstimatorVisualization>(
    //         "state_estimation/visualization", 10);

    m_service_node_position = this->create_service<march_shared_msgs::srv::GetNodePosition>(
        "state_estimation/get_node_position",
        std::bind(&RobotDescriptionNode::handleNodePositionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);
    m_service_node_jacobian = this->create_service<march_shared_msgs::srv::GetNodeJacobian>(
        "state_estimation/get_node_jacobian",
        std::bind(&RobotDescriptionNode::handleNodeJacobianRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    RCLCPP_INFO(this->get_logger(), "Robot Description Node initialized");
}

RobotDescriptionNode::~RobotDescriptionNode()
{
    RCLCPP_WARN(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode has been stopped.");
}

void RobotDescriptionNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::stateEstimationCallback");

    if (msg->joint_state.name.empty() || msg->joint_state.position.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"),
            "RobotDescriptionNode::stateEstimationCallback: Joint state is empty");
        return;
    }

    std::unordered_map<std::string, double> joint_positions;
    std::transform(msg->joint_state.name.begin(), msg->joint_state.name.end(), msg->joint_state.position.begin(),
        std::inserter(joint_positions, joint_positions.end()), [](const std::string& name, const double& position) {
            return std::make_pair(name, position);
        });
    publishVisualization(joint_positions);
}

void RobotDescriptionNode::publishVisualization(const std::unordered_map<std::string, double>& joint_positions)
{
    RCLCPP_WARN(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::publishVisualization");

    std::vector<Eigen::Vector3d> node_positions = m_robot_description->getAllNodesPosition(joint_positions);
    std::vector<Eigen::Matrix3d> node_rotations = m_robot_description->getAllNodesRotation(joint_positions);

    march_shared_msgs::msg::StateEstimatorVisualization state_estimator_visualization_msg;
    state_estimator_visualization_msg.node_names = m_robot_description->getAllNodeNames();
    state_estimator_visualization_msg.parent_node_names = m_robot_description->getAllParentNames();

    for (long unsigned int i = 0; i < node_positions.size(); i++) {
        geometry_msgs::msg::Pose node_pose;
        node_pose.position.x = node_positions[i].x();
        node_pose.position.y = node_positions[i].y();
        node_pose.position.z = node_positions[i].z();

        Eigen::Quaterniond quaternion(node_rotations[i]);
        node_pose.orientation.x = quaternion.x();
        node_pose.orientation.y = quaternion.y();
        node_pose.orientation.z = quaternion.z();
        node_pose.orientation.w = quaternion.w();

        state_estimator_visualization_msg.node_poses.push_back(node_pose);
    }

    m_publisher_state_estimator_visualization->publish(state_estimator_visualization_msg);
}

void RobotDescriptionNode::handleNodePositionRequest(
    const std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Response> response)
{
    // Assert that the request is not empty
    if (request->node_names.empty() || request->joint_names.empty() || request->joint_positions.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"),
            "RobotDescriptionNode::handleNodePositionRequest: Request is empty");
        return;
    }

    // Find a way to optimize this
    std::unordered_map<std::string, double> joint_positions;
    for (long unsigned int i = 0; i < request->joint_names.size(); i++) {
        joint_positions[request->joint_names[i]] = request->joint_positions[i];
    }

    std::vector<std::shared_ptr<RobotNode>> robot_nodes = m_robot_description->findNodes(request->node_names);
    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::handleNodePositionRequest: %d",
        robot_nodes.size());

    for (auto& robot_node : robot_nodes) {
        Eigen::Vector3d position = robot_node->getGlobalPosition(joint_positions);
        Eigen::Quaterniond orientation = Eigen::Quaterniond(robot_node->getGlobalRotation(joint_positions));

        if (request->reference_frame == "inertial") {
            Eigen::Quaterniond inertial_quaternion = m_robot_description->getInertialOrientation();
            position.noalias() = inertial_quaternion.toRotationMatrix() * position;
            orientation = inertial_quaternion * orientation;
        }

        geometry_msgs::msg::Pose node_pose;
        node_pose.position.x = position.x();
        node_pose.position.y = position.y();
        node_pose.position.z = position.z();
        node_pose.orientation.x = orientation.x();
        node_pose.orientation.y = orientation.y();
        node_pose.orientation.z = orientation.z();
        node_pose.orientation.w = orientation.w();

        response->node_poses.push_back(node_pose);
    }
    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::handleNodePositionRequest done");
}

void RobotDescriptionNode::handleNodeJacobianRequest(
    const std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Response> response)
{
    // Assert that the request is not empty
    if (request->node_names.empty() || request->joint_names.empty() || request->joint_positions.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("state_estimator_node"),
            "RobotDescriptionNode::handleNodePositionRequest: Request is empty");
        return;
    }

    // Find a way to optimize this
    std::unordered_map<std::string, double> joint_positions;
    for (long unsigned int i = 0; i < request->joint_names.size(); i++) {
        joint_positions[request->joint_names[i]] = request->joint_positions[i];
    }

    std::vector<std::shared_ptr<RobotNode>> robot_nodes = m_robot_description->findNodes(request->node_names);
    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::handleNodeJacobianRequest: %d",
        robot_nodes.size());
    std::vector<march_shared_msgs::msg::NodeJacobian> node_jacobians;

    for (auto& robot_node : robot_nodes) {
        march_shared_msgs::msg::NodeJacobian node_jacobian_msg;

        // TODO: Create a function that returns the joint names of a node in robot_node.hpp
        node_jacobian_msg.joint_names = robot_node->getJointNames();
        std::vector<std::string> joint_names = robot_node->getJointNames();
        RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::handleNodeJacobianRequest: %d",
            joint_names.size());

        Eigen::MatrixXd position_jacobian = robot_node->getGlobalPositionJacobian(joint_positions);
        Eigen::MatrixXd rotation_jacobian = robot_node->getGlobalRotationJacobian(joint_positions);

        if (request->reference_frame == "inertial") {
            Eigen::Matrix3d orientation_matrix = m_robot_description->getInertialOrientation().toRotationMatrix();
            position_jacobian.noalias() = orientation_matrix * position_jacobian;
            rotation_jacobian.noalias() = orientation_matrix * rotation_jacobian;
        }

        Eigen::MatrixXd jacobian(position_jacobian.rows() + rotation_jacobian.rows(), position_jacobian.cols());
        jacobian << position_jacobian, rotation_jacobian;

        std::vector<double> jacobian_vector(jacobian.data(), jacobian.data() + jacobian.size());
        node_jacobian_msg.rows = jacobian.rows();
        node_jacobian_msg.cols = jacobian.cols();
        node_jacobian_msg.jacobian = jacobian_vector;

        node_jacobians.push_back(node_jacobian_msg);
    }
    response->node_jacobians = node_jacobians;
    RCLCPP_DEBUG(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::handleNodeJacobianRequest done");
}