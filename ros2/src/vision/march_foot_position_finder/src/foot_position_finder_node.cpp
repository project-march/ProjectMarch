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

        std::cout << "A" << std::endl; 
        this->declare_parameter("realsense_simulation", false);

        this->declare_parameter("foot_gap", 0.0);
        this->declare_parameter("step_distance", 0.0);

        this->declare_parameter("outlier_distance", 0.0);
        this->declare_parameter("sample_size", 0);
        this->declare_parameter("height_zero_threshold", 0.0);

        this->declare_parameter("foot_width", 0.0);
        this->declare_parameter("foot_length", 0.0);
        this->declare_parameter("actual_foot_length", 0.0);

        this->declare_parameter("derivative_threshold", 0.0);
        this->declare_parameter("available_points_ratio", 0.0);
        this->declare_parameter("max_z_distance", 0.0);
        this->declare_parameter("num_track_points", 0);

        this->declare_parameter("displacements_outside", 0.0);
        this->declare_parameter("displacements_inside", 0.0);
        this->declare_parameter("displacements_near", 0.0);
        this->declare_parameter("displacements_far", 0.0);

        std::cout << "A" << std::endl; 

        left = new FootPositionFinder(this, "left");
        right = new FootPositionFinder(this, "right");

        this->set_on_parameters_set_callback(
            std::bind(&FootPositionFinderNode::parametersCallback, this,
                std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        left->startParameterCallback(parameters);
        left->startParameterCallback(parameters);

        // left = new
        // boost::thread(boost::bind(&FootPositionFinder::readParameters,
        //     positionFinderLeft, boost::ref(parameters)));
        // right = new
        // boost::thread(boost::bind(&FootPositionFinder::readParameters,
        //     positionFinderRight, boost::ref(parameters)));
        // left->detach();
        // right->detach();

        // for (const auto &parameter : parameters)
        // {
        //     if (parameter.get_name() == "my_str" &&
        //         parameter.get_type() ==
        //         rclcpp::ParameterType::PARAMETER_STRING)
        //     {
        //         my_str_ = parameter.as_string();
        //         RCLCPP_INFO(this->get_logger(), "Parameter 'my_str' changed:
        //         %s", my_str_.c_str());
        //     }
        // }
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
