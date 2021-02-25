#include <string>
#include <vector>

#include <ros/ros.h>
#include <march_realsense_reader/realsense_reader.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "march_realsense_reader");
  ros::NodeHandle n;

  RealSenseReader reader = RealSenseReader(&n);

  ros::AsyncSpinner spinner(2);  // Use n
  // threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
