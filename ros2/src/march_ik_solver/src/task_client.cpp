#include "march_ik_solver/task_client.hpp"

TaskClient::TaskClient() : Node("task_client")
{
    client_ = this->create_client<march_shared_msgs::srv::GetTaskReport>("get_task_report");
    send_request();

}

TaskClient::~TaskClient()
{
    // delete &client_;
}

void TaskClient::send_request()
{
    auto request = std::make_shared<march_shared_msgs::srv::GetTaskReport::Request>();

    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian: [%d]", result.get()->jacobian.size());
        for (int i = 0; i < result.get()->jacobian.size(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "jacobian[%d]: %f", i, result.get()->jacobian[i]);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service jacobian");
    }
}

void TaskClient::handle_response()
{
    // TODO
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TaskClient> node = std::make_shared<TaskClient>();
    rclcpp::shutdown();
    return 0;
}