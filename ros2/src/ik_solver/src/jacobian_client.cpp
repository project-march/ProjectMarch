#include "rclcpp/rclcpp.hpp"
#include "example_interface/srv/jacobian.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <cstdlib>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: jacobian_client m n");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("jacobian_client");
    rclcpp::Client<example_interface::srv::Jacobian>::SharedPtr client =
        node->create_client<example_interface::srv::Jacobian>("jacobian");
    
    auto request = std::make_shared<example_interface::srv::Jacobian::Request>();
    request->m = atoll(argv[1]);
    request->n = atoll(argv[2]);

    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jacobian: [%d]", result.get()->jacobis.size());
        for (int i = 0; i < result.get()->jacobis.size(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "jacobian[%d]: %f", i, result.get()->jacobis[i]);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service jacobian");
    }

    rclcpp::shutdown();
    return 0;
}