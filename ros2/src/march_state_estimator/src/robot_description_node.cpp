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

    // declare_parameter<std::string>("urdf_path", ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf");
    // std::string urdf_path = get_parameter("urdf_path").as_string();

    // m_robot_description = std::make_shared<RobotDescription>();
    // m_robot_description->parseURDF(urdf_path);
    // m_robot_description->configureRobotNodes();

    m_robot_description = robot_description;

    // TODO: Callback to state_estimation/state instead of /joint_states
    // m_joint_state_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_state_estimation_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // m_timer_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // m_node_positions_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // m_node_jacobian_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // m_joint_state_subscription_options.callback_group = m_joint_state_callback_group;
    m_state_estimation_subscription_options.callback_group = m_state_estimation_callback_group;

    // m_joint_state_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
    //     "joint_states", rclcpp::SensorDataQoS(), std::bind(&RobotDescriptionNode::jointStateCallback, this, std::placeholders::_1), m_joint_state_subscription_options);
    // m_state_estimation_subscription = this->create_subscription<march_shared_msgs::msg::StateEstimation>(
    //     "state_estimation/state", 1, std::bind(&RobotDescriptionNode::stateEstimationCallback, this, std::placeholders::_1), m_state_estimation_subscription_options);
    // m_state_visualization_publisher = this->create_publisher<march_shared_msgs::msg::StateEstimatorVisualization>("state_estimation/visualization", 1);
    // m_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RobotDescriptionNode::publishNodePositions, this), m_timer_callback_group);

    m_service_node_position = this->create_service<march_shared_msgs::srv::GetNodePosition>(
        "state_estimation/get_node_position", 
        std::bind(&RobotDescriptionNode::handleNodePositionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, m_node_positions_callback_group);
    m_service_node_jacobian = this->create_service<march_shared_msgs::srv::GetNodeJacobian>(
        "state_estimation/get_node_jacobian", 
        std::bind(&RobotDescriptionNode::handleNodeJacobianRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, m_node_jacobian_callback_group);

    // Temporary
    // m_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor done");
}

void RobotDescriptionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback");

    auto message = march_shared_msgs::msg::StateEstimatorVisualization();
    message.node_names = m_robot_description->getNodeNames();
    message.parent_node_names = m_robot_description->getParentNames();

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback: %d", msg->position.size());

    std::vector<Eigen::Vector3d> nodes_position = m_robot_description->getNodesPosition(msg->name, msg->position);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback: %d", nodes_position.size());
    std::vector<Eigen::Matrix3d> nodes_orientation = m_robot_description->getNodesRotation(msg->name, msg->position);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback: %d", nodes_orientation.size());

    for (long unsigned int i = 0; i < message.node_names.size(); i++)
    {
        geometry_msgs::msg::Pose pose;

        pose.position.x = nodes_position[i](0);
        pose.position.y = nodes_position[i](1);
        pose.position.z = nodes_position[i](2);

        Eigen::Quaterniond quaternion = Eigen::Quaterniond(nodes_orientation[i]);
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();

        message.node_poses.push_back(pose);
    }

    m_state_visualization_publisher->publish(message);
}

void RobotDescriptionNode::stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg)
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::stateEstimationCallback");

    // Sert that the message is not empty
    if (msg->joint_state.name.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::stateEstimationCallback: Message is empty");
        return;
    }

    auto message = march_shared_msgs::msg::StateEstimatorVisualization();
    message.node_names = m_robot_description->getNodeNames();
    message.parent_node_names = m_robot_description->getParentNames();

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::stateEstimationCallback: %d", msg->joint_state.position.size());
    // Print joint name and position
    // for (long unsigned int i = 0; i < msg->joint_state.name.size(); i++)
    // {
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::stateEstimationCallback: %s, %f", msg->joint_state.name[i].c_str(), msg->joint_state.position[i]);
    // }

    // std::vector<Eigen::Vector3d> nodes_position = m_robot_description->getNodesPosition(msg->joint_state.name, msg->joint_state.position);
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::stateEstimationCallback: %d", nodes_position.size());
    // std::vector<Eigen::Matrix3d> nodes_orientation = m_robot_description->getNodesRotation(msg->joint_state.name, msg->joint_state.position);
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::stateEstimationCallback: %d", nodes_orientation.size());

    // for (long unsigned int i = 0; i < message.node_names.size(); i++)
    // {
    //     geometry_msgs::msg::Pose pose;

    //     pose.position.x = nodes_position[i](0);
    //     pose.position.y = nodes_position[i](1);
    //     pose.position.z = nodes_position[i](2);

    //     Eigen::Quaterniond quaternion = Eigen::Quaterniond(nodes_orientation[i]);
    //     pose.orientation.x = quaternion.x();
    //     pose.orientation.y = quaternion.y();
    //     pose.orientation.z = quaternion.z();
    //     pose.orientation.w = quaternion.w();

    //     message.node_poses.push_back(pose);
    // }

    // m_state_visualization_publisher->publish(message);
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::stateEstimationCallback done");
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