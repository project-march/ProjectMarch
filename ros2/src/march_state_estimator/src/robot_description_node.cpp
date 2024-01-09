#include "march_state_estimator/robot_description_node.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/msg/node_jacobian.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <functional>
#include <chrono>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

RobotDescriptionNode::RobotDescriptionNode()
: Node("robot_description")
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor");

    declare_parameter<std::string>("urdf_path", ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf");
    std::string urdf_path = get_parameter("urdf_path").as_string();

    m_robot_description = std::make_shared<RobotDescription>();
    m_robot_description->parseURDF(urdf_path);
    m_robot_description->configureRobotNodes();

    m_joint_state_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&RobotDescriptionNode::jointStateCallback, this, std::placeholders::_1));
    m_node_positions_publisher = this->create_publisher<march_shared_msgs::msg::StateEstimatorVisualization>("state_estimator/node_positions", 1);
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotDescriptionNode::publishNodePositions, this));

    m_service_task_report = this->create_service<march_shared_msgs::srv::GetTaskReport>(
        "state_estimator/get_task_report", 
        std::bind(&RobotDescriptionNode::handleTaskReportRequest, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor done");
    m_service_node_position = this->create_service<march_shared_msgs::srv::GetNodePosition>(
        "state_estimator/get_node_position", 
        std::bind(&RobotDescriptionNode::handleNodePositionRequest, this, std::placeholders::_1, std::placeholders::_2));
    m_service_node_jacobian = this->create_service<march_shared_msgs::srv::GetNodeJacobian>(
        "state_estimator/get_node_jacobian", 
        std::bind(&RobotDescriptionNode::handleNodeJacobianRequest, this, std::placeholders::_1, std::placeholders::_2));

    // Temporary
    m_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
}

void RobotDescriptionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback");

    auto message = march_shared_msgs::msg::StateEstimatorVisualization();
    message.node_names = m_robot_description->getNodeNames();
    message.parent_node_names = m_robot_description->getParentNames();

    for (auto & node_position : m_robot_description->getNodesPosition(msg->name, msg->position))
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node_position[0];
        pose.position.y = node_position[1];
        pose.position.z = node_position[2];
        message.node_poses.push_back(pose);
    }

    m_node_positions_publisher->publish(message);
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback done");

    // Temporary
    m_joint_state_msg = msg;
}

void RobotDescriptionNode::handleTaskReportRequest(const std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Response> response)
{
    std::vector<std::string> names = {"left_ankle", "right_ankle"};
    std::vector<RobotNode*> robot_nodes = m_robot_description->findNodes(names);
    uint8_t task_m = 6;
    uint8_t task_n = 8;
    uint8_t task_m_unit = (uint8_t) (task_m / names.size());
    uint8_t task_n_unit = (uint8_t) (task_n / names.size());

    for (auto & robot_node : robot_nodes)
    {
        Eigen::Vector3d pose = robot_node->getGlobalPosition(m_joint_state_msg->name, m_joint_state_msg->position);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest: %f %f %f", pose(0), pose(1), pose(2));
        response->current_pose.push_back(pose(0));  // Global position x
        response->current_pose.push_back(pose(1));  // Global position y
        response->current_pose.push_back(pose(2));  // Global position z

        Eigen::MatrixXd jacobian = robot_node->getGlobalPositionJacobian(m_joint_state_msg->name, m_joint_state_msg->position);
        // TODO: Switch rows and columns
        for (int i = 0; i < task_m_unit; i++)
        {
            for (int j = 0; j < task_n_unit; j++)
            {
                if (i < jacobian.rows() && j < jacobian.cols())
                {
                    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest: %f", jacobian(i, j));
                    response->jacobians.push_back(jacobian(i, j));
                }
                else
                {
                    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest: %f", 0.0);
                    response->jacobians.push_back(0.0);
                }
                // TODO: Replace with Eigen::Map + add additional fields in GetTaskReport.srv (m and n per node)
            }
        }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest done");
}

void RobotDescriptionNode::handleNodePositionRequest(const std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetNodePosition::Response> response)
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleNodePositionRequest");
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDescriptionNode>());
    rclcpp::shutdown();
    return 0;
}