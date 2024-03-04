#include "footstep_generator/footstep_generator_m9.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootstepGenerator>());

    rclcpp::shutdown();
    return 0;
}