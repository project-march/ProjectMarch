#include <realsense_test_publisher.h>
#include <ros/ros.h>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_realsense_datasets_publisher");
    ros::NodeHandle n;

    RealsenseTestPublisher test_publisher = RealsenseTestPublisher(&n);

    ros::spin();
    ros::shutdown();
    return 0;
}
