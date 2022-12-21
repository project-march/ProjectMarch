#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "mujoco_interfaces/msg/ipd_input.hpp"
#include "march_shared_msgs/msg/gait_type.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GaitCommand:public rclcpp::Node {
public:
    GaitCommand()
        : Node("gait_command_node") {
        client = this->create_client<march_shared_msgs::srv::GaitCommand>("gait_command_client");
        ipd_subscriber = this->create_subscription<mujoco_interfaces::msg::IpdInput>("ipd_command", 10,
            std::bind(&GaitCommand::ipd_callback, this, _1));
        gait_type_publisher = this->create_publisher<march_shared_msgs::msg::GaitType>("gait_type", 10);
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
    };

private:
    void ipd_callback(mujoco_interfaces::msg::IpdInput::SharedPtr msg) {
        send_request(msg->input_cmd);
    };

    void send_request(int input_cmd){
        request->gait_type = input_cmd;
        future = client->async_send_request(request, std::bind(&GaitCommand::publish_gait_type, this, _1));
    }

    void publish_gait_type(rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedFuture response)
    {
        if (response.get()->success)
        {
            auto msg = march_shared_msgs::msg::GaitType();
            msg.gait_type = request->gait_type;
            gait_type_publisher->publish(msg);
        } else{
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Request was not a success");
        }
    }

    rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedPtr client;
    rclcpp::Subscription<mujoco_interfaces::msg::IpdInput>::SharedPtr ipd_subscriber;
    rclcpp::Publisher<march_shared_msgs::msg::GaitType>::SharedPtr gait_type_publisher;
    rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedFuture future;
    march_shared_msgs::srv::GaitCommand::Request::SharedPtr request;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitCommand>());
    rclcpp::shutdown();
    return 0;
}
