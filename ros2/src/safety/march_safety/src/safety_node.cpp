// Copyrihgt 2018 Project March
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "march_safety/safety_node.hpp"
#include "march_shared_msgs/msg/error.hpp"
#include "march_utility/node_utils.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}

SafetyNode::SafetyNode()
    : Node("safety_node", "march")
{
    error_subscriber
        = create_subscription<march_shared_msgs::msg::Error>(
            "/march/error", 20, std::bind(&SafetyNode::errorCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to /march/error");

    
}

void SafetyNode::startTimer()
{
    // Ensure that the safety node is updated every 50 ms
    rclcpp::TimerBase::SharedPtr timer = create_wall_timer(50ms, std::bind(&SafetyNode::update, this));
}

void SafetyNode::update()
{
    // Perform safety checks and update the system accordingly

    // Example: Check if there is any error message received
    // if (error_message_received)
    // {
    //     // Process the error message
    //     // processErrorMessage();
    // }
}

/**
 * @brief Callback function for the /march/error topic
 * @param msg The received error message
 */
void SafetyNode::errorCallback(const march_shared_msgs::msg::Error::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received error message: %s", msg->error_message.c_str());
}
