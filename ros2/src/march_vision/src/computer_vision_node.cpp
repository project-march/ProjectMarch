//#include <processing/camera_interface.h>
//#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <librealsense2/rs.hpp>
#include <thread>

//#include <elevation_mapping/elevation_mapping.hpp

// TODO: Make Life cycle node?
//class ComputerVisionNode : public rclcpp::Node {

// public:
//     CamerasInterface* left;
//     CamerasInterface* right;

//     ComputerVisionNode() : Node("march_computer_vision",
//                            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
//                            /*automatically_declare_parameters_from_overrides=*/true)) {

//         // TODO: Combine into one?
//         left = new CameraInterface(this, "left");
//         right = new CameraInterface(this, "right");

//         this->set_on_parameters_set_callback(
//             std::bind(&ComputerVisionNode::parametersCallback, this, std::placeholders::_1));
//     }

//     rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
//         // TODO: Change the node threads
//         std::thread thread_left(&CamerasInterface::readParameters, left, parameters);
//         thread_left.detach();
//         std::thread thread_right(&CameraInterface::readParameters, right, parameters);
//         thread_right.detach();

//         rcl_interfaces::msg::SetParametersResult result;
//         result.successful = true;
//         result.reason = "success";
//         return result;
//     }
// };

int main(int argc, char** argv)
{
    // rclcpp::init(argc, argv);

    // auto nodeHandle = rclcpp::Node::make_shared("elevation_mapping");

    // nodeHandle->declare_parameter("num_callback_threads", 1);
    // nodeHandle->declare_parameter("postprocessor_num_threads", 1);

    // elevation_mapping::ElevationMapping elevationMap(nodeHandle);  

    // // Spin
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), nodeHandle->get_parameter("num_callback_threads").as_int());
    // executor.add_node(nodeHandle);
    // executor.spin();

    // // RCLCPP_INFO(nodeHandle->get_logger(), "Trying to spin node");
    // // rclcpp::spin(nodeHandle);
    // RCLCPP_INFO(nodeHandle->get_logger(), "Spinning node");
    // rclcpp::shutdown();  
    return 0;
}
