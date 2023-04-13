#include "torque_converter/torque_converter.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TorqueConverter>());
    rclcpp::shutdown();
    return 0;
}
