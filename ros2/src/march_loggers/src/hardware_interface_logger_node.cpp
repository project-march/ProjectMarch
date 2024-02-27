#include "rclcpp/rclcpp.hpp"
#include <rcl_interfaces/msg/log.hpp>

class MyNode : public rclcpp::Node
{
public:
    MyNode()
    : Node("HardwareInterfaceLogger")
    {
        RCLCPP_ERROR_ONCE(get_logger(), "This log comes from this->get_logger()");
        RCLCPP_ERROR_ONCE(rclcpp::get_logger("notanode"), "This log comes from get_logger(\"notanode\")");
        RCLCPP_ERROR_ONCE(rclcpp::get_logger("rclcpp"), "This log comes from get_logger(\"rclcpp\")");
        
        publisher_ = this->create_publisher<rcl_interfaces::msg::Log>("/rosout", 10);
        
        // Create a log message
        auto log_msg = std::make_shared<rcl_interfaces::msg::Log>();
        log_msg->stamp = now();
        log_msg->level = rcl_interfaces::msg::Log::ERROR;
        log_msg->name = "hardware_interface_logger_node";
        log_msg->msg = "This log comes from the rosout publisher";
        log_msg->file = "hardware_interface_logger_node.cpp";
        log_msg->function = "rclcpp:spin(node)";
        log_msg->line = 5;

        // Publish the log message 3 times with a 1 second interval
        for (int i = 0; i < 6; ++i) {
            publisher_->publish(*log_msg);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

    RCLCPP_ERROR_ONCE(get_logger(), "This log comes from this->get_logger()");


    }

private:
    rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}