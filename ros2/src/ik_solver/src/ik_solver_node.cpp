#include "ik_solver/ik_solver.hpp"
#include <iostream>



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin_once(std::make_shared<>());

    rclcpp::shutdown();
    return 0;
}
