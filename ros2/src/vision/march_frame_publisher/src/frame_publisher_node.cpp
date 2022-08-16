/**
 * @author Jelmer de Wolde, Tuhin Das - MARCH 7
 */

#include "rclcpp/rclcpp.hpp"
#include <frame_publisher.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<FramePublisher>();
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
