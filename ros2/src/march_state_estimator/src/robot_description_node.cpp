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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor");

    m_robot_description = robot_description;

    // m_node_positions_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // m_node_jacobian_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // m_service_node_position = this->create_service<march_shared_msgs::srv::GetNodePosition>(
    //     "state_estimation/get_node_position", 
    //     std::bind(&RobotDescriptionNode::handleNodePositionRequest, this, std::placeholders::_1, std::placeholders::_2),
    //     rmw_qos_profile_services_default, m_node_positions_callback_group);
    // m_service_node_jacobian = this->create_service<march_shared_msgs::srv::GetNodeJacobian>(
    //     "state_estimation/get_node_jacobian", 
    //     std::bind(&RobotDescriptionNode::handleNodeJacobianRequest, this, std::placeholders::_1, std::placeholders::_2),
    //     rmw_qos_profile_services_default, m_node_jacobian_callback_group);

    // // Declare parameters
    // declare_parameter("names", std::vector<std::string>());

    // // Get parameters
    // std::vector<std::string> names = get_parameter("names").as_string_array();

    // // Print parameters
    // RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::configureParameters: %d", names.size());
    // for (long unsigned int i = 0; i < names.size(); i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::configureParameters: %s", names[i].c_str());
    // }

    // // Get config file path
    // std::string config_file_path = ament_index_cpp::get_package_share_directory("march_state_estimator") + "/config/robot_definition-config.yaml";
    // RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::configureParameters: %s", config_file_path.c_str());

    // // Parse config file
    // YAML::Node config_file = YAML::LoadFile(config_file_path);
    // // const std::string test = config_file["L_UL"]["linear"]["position"]["x"].as<std::string>();
    // // RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::configureParameters: %s", test.c_str());

    // const std::vector<std::string> names = config_file["names"].as<std::vector<std::string>>();
    // for (long unsigned int i = 0; i < names.size(); i++)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::configureParameters: %s", names[i].c_str());
    //     const std::string abs_linear_position = config_file[names[i]]["linear"]["position"]["x"].as<std::string>();
    //     RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::configureParameters: %s", abs_linear_position.c_str());
    // }

    m_service_node_position = this->create_service<march_shared_msgs::srv::GetNodePosition>(
        "state_estimation/get_node_position", 
        std::bind(&RobotDescriptionNode::handleNodePositionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);
    m_service_node_jacobian = this->create_service<march_shared_msgs::srv::GetNodeJacobian>(
        "state_estimation/get_node_jacobian",
        std::bind(&RobotDescriptionNode::handleNodeJacobianRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor done");
}

void RobotDescriptionNode::handleNodePositionRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Response> response)
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest");

    // Assert that the request is not empty
    if (request->node_names.empty() || request->joint_names.empty() || request->joint_positions.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: Request is empty");
        return;
    }

    // // Print request
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: %d", request->node_names.size());
    // for (long unsigned int i = 0; i < request->joint_names.size(); i++)
    // {
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: %s, %f", request->joint_names[i].c_str(), request->joint_positions[i]);
    // }

    std::vector<RobotNode*> robot_nodes = m_robot_description->findNodes(request->node_names);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: %d", robot_nodes.size());
    // for (auto & robot_node : robot_nodes)
    // {
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: node %s", robot_node->getName().c_str());
    // }

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
    if (request->node_names.empty() || request->joint_names.empty() || request->joint_positions.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest: Request is empty");
        return;
    }

    // // Print request
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %d", request->node_names.size());
    // for (long unsigned int i = 0; i < request->joint_names.size(); i++)
    // {
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %s, %f", request->joint_names[i].c_str(), request->joint_positions[i]);
    // }

    std::vector<RobotNode*> robot_nodes = m_robot_description->findNodes(request->node_names);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %d", robot_nodes.size());
    // for (auto & robot_node : robot_nodes)
    // {
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: node %s", robot_node->getName().c_str());
    // }
    std::vector<march_shared_msgs::msg::NodeJacobian> node_jacobians;

    for (auto & robot_node : robot_nodes)
    {
        march_shared_msgs::msg::NodeJacobian node_jacobian_msg;

        // TODO: Create a function that returns the joint names of a node in robot_node.hpp
        node_jacobian_msg.joint_names = robot_node->getJointNames();
        std::vector<std::string> joint_names = robot_node->getJointNames();
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %d", joint_names.size());

        Eigen::MatrixXd jacobian = robot_node->getGlobalPositionJacobian(request->joint_names, request->joint_positions);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %d %d", jacobian.rows(), jacobian.cols());
        // for (int i = 0; i < jacobian.rows(); i++)
        // {
        //     for (int j = 0; j < jacobian.cols(); j++)
        //     {
        //         RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest: %s, %f", joint_names[j].c_str(), jacobian(i, j));
        //     }
        // }
        for (int i = 0; i < jacobian.rows(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::handleNodeJacobianRequest %s: %f, %f, %f, %f",
                robot_node->getName().c_str(), jacobian(i, 0), jacobian(i, 1), jacobian(i, 2), jacobian(i, 3));
        }

        node_jacobian_msg.rows = jacobian.rows();
        node_jacobian_msg.cols = jacobian.cols();

        std::vector<double> jacobian_vector(jacobian.data(), jacobian.data() + jacobian.size());
        node_jacobian_msg.jacobian = jacobian_vector;

        for (long unsigned int i = 0; i < jacobian_vector.size(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("state_estimator_node"), "RobotDescriptionNode::handleNodeJacobianRequest: %f", jacobian_vector[i]);
        }

        node_jacobians.push_back(node_jacobian_msg);
    }

    response->node_jacobians = node_jacobians;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodeJacobianRequest done");
}