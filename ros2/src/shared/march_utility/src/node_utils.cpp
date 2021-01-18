// Copyright 2020 Project March.
#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/srv/get_joint_names.hpp"
#include "march_utility/node_utils.hpp"

#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace node_utils
{
    /*
     * @brief Get the Joint names from the robot information node.
     * @param node Node to create client for and use logger of.
     * @return Returns the list of joint names.
     */
      std::vector<std::string> get_joint_names(rclcpp::Node& node)
      {
        std::vector<std::string> names;

        auto client = node.create_client<march_shared_msgs::srv::GetJointNames>("/march/robot_information/get_joint_names");
        auto request = std::make_shared<march_shared_msgs::srv::GetJointNames::Request>();

        // Wait until the service is available. Sending a request to an unavailable service
        // will always fail.
        while (!client->wait_for_service(5s))
        {
          if (!rclcpp::ok())
          {
            RCLCPP_ERROR(node.get_logger(), "Interrupted while waiting for get_joint_names service. Exiting.");
            return names;
          }
          RCLCPP_INFO(node.get_logger(), "The get_joint_names service was unavailable, waiting again...");
        }

        // Send the request and push the received names to the names vector.
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node.get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
          names = std::move(result.get()->joint_names);
        }
        else
        {
          RCLCPP_ERROR(node.get_logger(), "Failed to call get_joint_names service");
        }

        return names;
      }
} // namespace node_utils


