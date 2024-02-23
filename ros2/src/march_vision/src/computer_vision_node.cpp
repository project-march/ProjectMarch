#include <processing/cameras_interface.h>
//#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>

// TODO: Make Life cycle node?
class ComputerVisionNode : public rclcpp::Node {

public:
    CamerasInterface* left;
    CamerasInterface* right;

    ComputerVisionNode() : Node("march_computer_vision",
                           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
                           /*automatically_declare_parameters_from_overrides=*/true)) {

        // TODO: Combine into one?
        left = new CamerasInterface(this, "left");
        right = new CamerasInterface(this, "right");

        this->set_on_parameters_set_callback(
            std::bind(&ComputerVisionNode::parametersCallback, this, std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
        // TODO: Change the node threads
        std::thread thread_left(&CamerasInterface::readParameters, left, parameters);
        thread_left.detach();
        std::thread thread_right(&CamerasInterface::readParameters, right, parameters);
        thread_right.detach();

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        return result;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<ComputerVisionNode>();
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
