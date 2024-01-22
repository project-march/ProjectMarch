#include "march_state_estimator/robot_description_node.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/node_jacobian.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <functional>
#include <chrono>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

RobotDescriptionNode::RobotDescriptionNode(std::shared_ptr<RobotDescription> robot_description)
: Node("robot_description")
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor");

    m_robot_description = robot_description;

    m_node_positions_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_node_jacobian_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_service_node_position = this->create_service<march_shared_msgs::srv::GetNodePosition>(
        "state_estimation/get_node_position", 
        std::bind(&RobotDescriptionNode::handleNodePositionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, m_node_positions_callback_group);
    m_service_node_jacobian = this->create_service<march_shared_msgs::srv::GetNodeJacobian>(
        "state_estimation/get_node_jacobian", 
        std::bind(&RobotDescriptionNode::handleNodeJacobianRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, m_node_jacobian_callback_group);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor done");
}

void RobotDescriptionNode::handleNodePositionRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Response> response)
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest");

    // Assert that the request is not empty
    if (request->node_names.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: Request is empty");
        return;
    }

    std::vector<RobotNode*> robot_nodes = m_robot_description->findNodes(request->node_names);

    for (auto & robot_node : robot_nodes)
    {
        Eigen::Vector3d pose = robot_node->getGlobalPosition(request->joint_names, request->joint_positions);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: %f %f %f", pose(0), pose(1), pose(2));
        
        geometry_msgs::msg::Point node_position;
        node_position.x = pose(0);
        node_position.y = pose(1);
        node_position.z = pose(2);

        response->node_positions.push_back(node_position);
    }

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest done");
}

void RobotDescriptionNode::handleNodeJacobianRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetNodeJacobian::Response> response)
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest");

    // Assert that the request is not empty
    if (request->node_names.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: Request is empty");
        return;
    }

    std::vector<RobotNode*> robot_nodes = m_robot_description->findNodes(request->node_names);
    std::vector<march_shared_msgs::msg::NodeJacobian> node_jacobians;

    for (auto & robot_node : robot_nodes)
    {
        march_shared_msgs::msg::NodeJacobian node_jacobian_msg;

        // TODO: Create a function that returns the joint names of a node in robot_node.hpp
        node_jacobian_msg.joint_names = robot_node->getJointNames();
        std::vector<std::string> joint_names = robot_node->getJointNames();

        Eigen::MatrixXd jacobian = robot_node->getGlobalPositionJacobian(request->joint_names, request->joint_positions);

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %d %d", jacobian.rows(), jacobian.cols());
        for (int i = 0; i < jacobian.rows(); i++)
        {
            for (int j = 0; j < jacobian.cols(); j++)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %s, %f", joint_names[j].c_str(), jacobian(i, j));
            }
        }

        node_jacobian_msg.rows = jacobian.rows();
        node_jacobian_msg.cols = jacobian.cols();

        std::vector<double> jacobian_vector(jacobian.data(), jacobian.data() + jacobian.size());
        node_jacobian_msg.jacobian = jacobian_vector;

        node_jacobians.push_back(node_jacobian_msg);
    }

    response->node_jacobians = node_jacobians;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest done");
}