#include <iostream>
#include <librealsense2/rs.hpp>
#include "rclcpp/rclcpp.hpp"
// #include "foot_position_finder.h"

class FootPositionFinderNode : public rclcpp::Node {
	public:
		FootPositionFinderNode()
		: Node("march_foot_position_finder")
		{
			this->declare_parameter("physical_cameras");
			this->declare_parameter("base_frame");

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
		}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor exec;
	auto node = std::make_shared<FootPositionFinderNode>();
	exec.add_node(node);

	// FootPositionFinder left = new FootPositionFinder(&node, "left");
	// FootPositionFinder right = new FootPositionFinder(&node, "right");

	exec.spin();

	rclcpp::shutdown();

    return 0;
}
