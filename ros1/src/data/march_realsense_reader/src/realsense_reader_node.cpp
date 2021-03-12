#include <string>
#include <vector>
#include <ros/ros.h>
#include <march_realsense_reader/realsense_reader.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "march_realsense_reader");
  ros::NodeHandle n;

  RealSenseReader reader = RealSenseReader(&n);

  ros::spin();
  ros::shutdown();
  return 0;
}
