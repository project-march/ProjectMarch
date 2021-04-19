#include <march_realsense_reader/realsense_reader.h>
#include <ros/ros.h>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_realsense_reader");
    ros::NodeHandle n;

    RealSenseReader reader = RealSenseReader(&n);

    ros::spin();
    ros::shutdown();
    return 0;
}
