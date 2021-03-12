#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <march_realsense_reader/realsense_reader.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "march_realsense_reader");
  ros::NodeHandle n;

  RealSenseReader reader = RealSenseReader(&n);
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::spin();
  ros::shutdown();
  return 0;
}
