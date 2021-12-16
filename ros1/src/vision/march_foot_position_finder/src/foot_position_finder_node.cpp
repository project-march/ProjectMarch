#include "foot_position_finder.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_foot_position_finder");

    ros::NodeHandle n;
    FootPositionFinder pointFinderLeft(&n, false, "left");
    FootPositionFinder pointFinderRight(&n, false, "right");

    ros::spin();
    ros::shutdown();

    return 0;
}