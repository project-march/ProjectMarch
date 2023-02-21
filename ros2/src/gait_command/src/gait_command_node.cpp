#include "march_shared_msgs/msg/gait_type.hpp"
#include "mujoco_interfaces/msg/ipd_input.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include <chrono>
#include <cstdlib>
#include <memory>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GaitCommand : public rclcpp::Node {
public:
    GaitCommand()
        : Node("gait_command_node")
    {
        ipd_subscriber = this->create_subscription<mujoco_interfaces::msg::IpdInput>(
            "ipd_command", 10, std::bind(&GaitCommand::ipd_callback, this, _1));
        gait_type_publisher = this->create_publisher<march_shared_msgs::msg::GaitType>("gait_type", 10);
    };

private:
    void ipd_callback(mujoco_interfaces::msg::IpdInput::SharedPtr msg)
    {
        auto gait_msg = march_shared_msgs::msg::GaitType();
        gait_msg.gait_type = msg->input_cmd;
        gait_type_publisher->publish(gait_msg);
    };
    rclcpp::Subscription<mujoco_interfaces::msg::IpdInput>::SharedPtr ipd_subscriber;
    rclcpp::Publisher<march_shared_msgs::msg::GaitType>::SharedPtr gait_type_publisher;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitCommand>());
    rclcpp::shutdown();
    return 0;
}
