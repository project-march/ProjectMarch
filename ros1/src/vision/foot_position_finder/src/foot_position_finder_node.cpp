#include "foot_position_finder.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "foot_position_finder");

    ros::NodeHandle n;
    FootPositionFinder pointFinderLeft = FootPositionFinder(&n, false, 'l');
    FootPositionFinder pointFinderRight = FootPositionFinder(&n, false, 'r');
    
    ros::spin();
    ros::shutdown();

    return 0;
}
