#include "march_shared_msgs/msg/gait_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <string>

using std::placeholders::_1;

class StateMachine : public rclcpp::Node {
public:
    StateMachine()
        : Node("state_machine_node")
    {
        gait_command_subscriber = this->create_subscription<march_shared_msgs::msg::GaitType>(
            "gait_type", 10, std::bind(&StateMachine::gait_command_callback, this, _1));
    }

private:
    void gait_command_callback(march_shared_msgs::msg::GaitType::SharedPtr msg)
    {
        // TODO Implement safety check for invalid gait type transition
        //  and define a unified way to represent those transitions as ints.\
        // auto gait_command = msg->gait_type;
    }

    rclcpp::Subscription<march_shared_msgs::msg::GaitType>::SharedPtr gait_command_subscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachine>());
    rclcpp::shutdown();
    return 0;
}
