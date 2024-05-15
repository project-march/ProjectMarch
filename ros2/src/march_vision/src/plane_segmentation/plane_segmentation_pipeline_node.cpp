#include <rclcpp/rclcpp.hpp>
#include "march_vision/plane_segmentation/plane_segmentation_pipeline.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
     
  auto nodeHandle = rclcpp::Node::make_shared("plane_segmentation_pipeline");

  nodeHandle->declare_parameter("frequency_plane_segmentation_pipeline", 15.0); // Can bump up to 20 Hz probably
  double frequency_plane_segmentation_pipeline;
  nodeHandle->get_parameter("frequency_plane_segmentation_pipeline", frequency_plane_segmentation_pipeline);

  rclcpp::Rate rate(frequency_plane_segmentation_pipeline);
  while (rclcpp::ok()) {
    rclcpp::spin_some(nodeHandle);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
