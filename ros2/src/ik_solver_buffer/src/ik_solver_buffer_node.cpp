#include "ik_solver_buffer/ik_solver_buffer.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BufferNode>());
    rclcpp::shutdown();
    return 0;
}
