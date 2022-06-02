#include "foot_position_finder.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <librealsense2/rs.hpp>

class FootPositionFinderNode : public rclcpp::Node {
public:
    FootPositionFinder* left;
    FootPositionFinder* right;

    FootPositionFinderNode()
        : Node("march_foot_position_finder")
    {

        this->declare_parameter("realsense_simulation");

        this->declare_parameter("foot_gap");
        this->declare_parameter("step_distance");

        this->declare_parameter("outlier_distance");
        this->declare_parameter("sample_size");
        this->declare_parameter("height_zero_threshold");

        this->declare_parameter("foot_width");
        this->declare_parameter("foot_length");
        this->declare_parameter("actual_foot_length");

        this->declare_parameter("derivative_threshold");
        this->declare_parameter("available_points_ratio");
        this->declare_parameter("max_z_distance");
        this->declare_parameter("num_track_points");

        this->declare_parameter("displacements_outside");
        this->declare_parameter("displacements_inside");
        this->declare_parameter("displacements_near");
        this->declare_parameter("displacements_far");

        left = new FootPositionFinder(this, "left");
        // right = new FootPositionFinder(this, "right");

        this->set_on_parameters_set_callback(
            std::bind(&FootPositionFinderNode::parametersCallback, this,
                std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        left->startParameterCallback(parameters);
        // right->startParameterCallback(parameters);

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<FootPositionFinderNode>();
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
