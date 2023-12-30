#include "march_state_estimator/robot_description_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <functional>
#include <chrono>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

RobotDescriptionNode::RobotDescriptionNode()
: Node("march_state_estimator_node")
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor");
    robot_description_ = std::make_shared<RobotDescription>();
    robot_description_->parseURDF(ament_index_cpp::get_package_share_directory("march_description") + "/urdf/march8/hennie_with_koen.urdf");
    robot_description_->configureRobotNodes();

    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "new_joint_states", 1, std::bind(&RobotDescriptionNode::jointStateCallback, this, std::placeholders::_1));
    node_positions_publisher_ = this->create_publisher<march_shared_msgs::msg::StateEstimatorVisualization>("node_positions", 1);
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotDescriptionNode::publishNodePositions, this));
    m_service_task_report = this->create_service<march_shared_msgs::srv::GetTaskReport>(
        "get_task_report", 
        std::bind(&RobotDescriptionNode::handleTaskReportRequest, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode constructor done");

    // Temporary
    m_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
}

void RobotDescriptionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback");

    auto message = march_shared_msgs::msg::StateEstimatorVisualization();
    message.node_names = robot_description_->getNodeNames();
    message.parent_node_names = robot_description_->getParentNames();

    for (auto & node_position : robot_description_->getNodesPosition(msg->name, msg->position))
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node_position[0];
        pose.position.y = node_position[1];
        pose.position.z = node_position[2];
        message.node_poses.push_back(pose);
    }

    node_positions_publisher_->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::jointStateCallback done");

    // Temporary
    m_joint_state_msg = msg;
}

void RobotDescriptionNode::handleTaskReportRequest(const std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Response> response)
{
    std::vector<std::string> names = {"left_ankle", "right_ankle"};
    std::vector<RobotNode*> robot_nodes = robot_description_->findNodes(names);
    uint8_t task_m = 6;
    uint8_t task_n = 8;
    uint8_t task_m_unit = (uint8_t) (task_m / names.size());
    uint8_t task_n_unit = (uint8_t) (task_n / names.size());

    // std::vector<double> current_pose;
    // std::vector<double> jacobians;

    for (auto & robot_node : robot_nodes)
    {
        Eigen::Vector3d pose = robot_node->getGlobalPosition(m_joint_state_msg->name, m_joint_state_msg->position);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest: %f %f %f", pose(0), pose(1), pose(2));
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
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest: %f", jacobian(i, j));
                    response->jacobians.push_back(jacobian(i, j));
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest: %f", 0.0);
                    response->jacobians.push_back(0.0);
                }
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::handleTaskReportRequest done");
}

// void RobotDescriptionNode::publishNodePositions()
// {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::publishNodePositions");
//     auto message = march_shared_msgs::msg::StateEstimatorVisualization();
//     message.node_names = robot_description_->getNodeNames();
//     message.parent_node_names = robot_description_->getParentNames();

//     for (auto & node_position : robot_description_->getNodesPosition())
//     {
//         geometry_msgs::msg::Pose pose;
//         pose.position.x = node_position[0];
//         pose.position.y = node_position[1];
//         pose.position.z = node_position[2];
//         message.node_poses.push_back(pose);
//     }

//     node_positions_publisher_->publish(message);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RobotDescriptionNode::publishNodePositions done");
// }

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDescriptionNode>());
    rclcpp::shutdown();
    return 0;
}