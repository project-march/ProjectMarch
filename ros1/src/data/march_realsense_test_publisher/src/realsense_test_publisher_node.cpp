#include <realsense_test_publisher.h>
#include <ros/ros.h>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_realsense_datasets_publisher");
    ros::NodeHandle n;

    RealsenseTestPublisher test_publisher = RealsenseTestPublisher(&n);

    ros::Timer timer_publish_pointcloud = n_->createTimer(ros::Duration(PUBLISH_RATE),
                                                          std::bind(&RealsenseTestPublisher::publishTestCloud,
                                                                    test_publisher));

    ros::spin();
    ros::shutdown();
    return 0;
}
