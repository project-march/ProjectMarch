#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "mujoco_interfaces/msg/ipd_input.hpp"
#include "march_shared_msgs/msg/gait_type.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
using std::placeholders::_1;

class GaitCommand:public rclcpp::Node {
public:
    GaitCommand()
        : Node("gait_command_node") {
        client = this->create_client<march_shared_msgs::srv::GaitCommand>("gait_command_client");
        ipd_subscriber = this->create_subscription<mujoco_interfaces::msg::IpdInput>("ipd_command", 10,
            std::bind(&GaitCommand::ipd_callback, this, _1));
        gait_type_publisher = this->create_publisher<march_shared_msgs::msg::GaitType>("gait_type", 10);
    };

private:
    void ipd_callback(mujoco_interfaces::msg::IpdInput::SharedPtr msg) {
        send_request(msg->input_cmd);
    };

    void send_request(int input_cmd){
        auto request = std::make_shared<march_shared_msgs::srv::GaitCommand::Request>();
        request->gait_type = input_cmd;

        while (!client->wait_for_service(1)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto message = march_shared_msgs::msg::GaitType();
            message.gait_type = request->gait_type;
            gait_type_publisher->publish(message);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service with gait_command");
        }
    }

    rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedPtr client;
    rclcpp::Subscription<mujoco_interfaces::msg::IpdInput>::SharedPtr ipd_subscriber;
    rclcpp::Publisher<march_shared_msgs::msg::GaitType>::SharedPtr gait_type_publisher;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitCommand>());
    rclcpp::shutdown();
    return 0;
}
