#include "march_ik_solver/current_joint_positions_server.hpp"

CurrentJointPositionServer::CurrentJointPositionServer()
    : Node("current_joint_position_server")
{
    service_ = this->create_service<march_shared_msgs::srv::GetCurrentJointPositions>("get_current_joint_positions",
        std::bind(&CurrentJointPositionServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Ready to get current joint positions.");
}

CurrentJointPositionServer::~CurrentJointPositionServer()
{
    // delete &service_;
}

void CurrentJointPositionServer::handle_request(
    const std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetCurrentJointPositions::Response> response)
{
    // Request current joint positions from the robot at the given timestamp.
    RCLCPP_INFO(this->get_logger(), "Incoming request");
    RCLCPP_INFO(this->get_logger(), "Current joint positions at timestamp: %f", request->timestamp);

    // Generate current joint positions.
    Eigen::VectorXd current_joint_positions = Eigen::VectorXd(8);
    current_joint_positions << 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0;
    std::vector<double> current_joint_positions_vector(
        current_joint_positions.data(), current_joint_positions.data() + current_joint_positions.size());

    // Generate current joint velocities.
    Eigen::VectorXd current_joint_velocities = Eigen::VectorXd(8);
    current_joint_velocities << 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0;
    std::vector<double> current_joint_velocities_vector(
        current_joint_velocities.data(), current_joint_velocities.data() + current_joint_velocities.size());

    // Response current joint positions and velocities.
    response->status = true;
    response->joint_positions = current_joint_positions_vector;
    response->joint_velocities = current_joint_velocities_vector;

    // Print current joint positions.
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%d]", response.get()->joint_positions.size());
    for (int i = 0; i < (int)response->joint_positions.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "current_joint_positions[%d]: %f", i, response->joint_positions[i]);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentJointPositionServer>());
    rclcpp::shutdown();
    return 0;
}