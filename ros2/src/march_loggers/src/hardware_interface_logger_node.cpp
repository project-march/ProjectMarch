#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode()
    : Node("HardwareInterface")
    {
        RCLCPP_ERROR_ONCE(get_logger(), "----------- WILL THIS BE LOGGED TO ROSOUT? -----------");
        RCLCPP_ERROR_ONCE(rclcpp::get_logger("notanode"), "----------- WILL THIS BE LOGGED TO ROSOUT? -----------");

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}