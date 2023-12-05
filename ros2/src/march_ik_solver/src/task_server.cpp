#include "march_ik_solver/task_server.hpp"

TaskServer::TaskServer()
    : Node("task_server")
{
    service_ = this->create_service<march_shared_msgs::srv::GetTaskReport>(
        "get_task_report", std::bind(&TaskServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Ready to generate jacobian.");
}

TaskServer::~TaskServer()
{
    // delete &service_;
}

void TaskServer::handle_request(const std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Request> request,
    std::shared_ptr<march_shared_msgs::srv::GetTaskReport::Response> response)
{
    int task_id = request->task_id;
    Eigen::MatrixXd jacobian = Eigen::MatrixXd(6, 8);
    double counter = 0.0;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 8; j++) {
            jacobian(i, j) = (double)(j + 1);
            // counter = counter + 1e-3;
        }
    }
    // jacobian.resize(1, jacobian.size());
    Eigen::VectorXd jacobian_vector = Eigen::Map<Eigen::VectorXd>(jacobian.data(), jacobian.size());
    for (int i = 0; i < jacobian_vector.size(); i++) {
        response->jacobian.push_back(jacobian_vector(i));
    }
    RCLCPP_INFO(this->get_logger(), "Incoming request\nm: %d\nn: %d", 6, 8);
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%d]", response.get()->jacobian.size());
    for (int i = 0; i < response->jacobian.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "jacobian[%d]: %f", i, response->jacobian[i]);
    }

    Eigen::VectorXd current_pose = Eigen::VectorXd(6);
    current_pose << 1.0, 2.0, 3.0, 1.0, 2.0, 3.0;
    for (int i = 0; i < current_pose.size(); i++) {
        response->current_pose.push_back(current_pose(i));
    }
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%d]", response.get()->current_pose.size());
    for (int i = 0; i < response->current_pose.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "current_pose[%d]: %f", i, response->current_pose[i]);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskServer>());
    rclcpp::shutdown();
    return 0;
}