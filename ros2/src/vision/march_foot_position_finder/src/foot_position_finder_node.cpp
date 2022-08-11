#include "foot_position_finder.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>

class FootPositionFinderNode : public rclcpp::Node {
public:
    FootPositionFinder* left;
    FootPositionFinder* right;

    FootPositionFinderNode()
        : Node("march_foot_position_finder",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
                /*automatically_declare_parameters_from_overrides=*/true))
    {
        left = new FootPositionFinder(this, "left");
        right = new FootPositionFinder(this, "right");

        this->set_on_parameters_set_callback(
            std::bind(&FootPositionFinderNode::parametersCallback, this, std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
    {
        std::thread thread_left(&FootPositionFinder::readParameters, left, parameters);
        thread_left.detach();
        std::thread thread_right(&FootPositionFinder::readParameters, right, parameters);
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

    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<FootPositionFinderNode>();
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
