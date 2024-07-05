/*
 * P.Fankhauser, M.Bloesch, and M.Hutter, 
 * "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization",
 * in IEEE Robotics and Automation Letters (RA-L)
 *
 * MARCH functionality and ROS2 porting by Alexander Andonov
 */

#include <rclcpp/rclcpp.hpp>
#include "elevation_mapping/elevation_mapping.hpp"

namespace elevation_mapping {

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
     
  auto nodeHandle = rclcpp::Node::make_shared("elevation_mapping");

  nodeHandle->declare_parameter("num_callback_threads", 1);
  // TODO: Fix ROS2 porting for filter chain, when using more threads
  nodeHandle->declare_parameter("postprocessor_num_threads", 1);

  ElevationMapping elevationMap(nodeHandle);  
    
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), nodeHandle->get_parameter("num_callback_threads").as_int());
  executor.add_node(nodeHandle);
  executor.spin();
  RCLCPP_INFO(nodeHandle->get_logger(), "Spinning elevation_mapping node");
  rclcpp::shutdown();  
  return 0;
}
}  // namespace elevation_mapping
