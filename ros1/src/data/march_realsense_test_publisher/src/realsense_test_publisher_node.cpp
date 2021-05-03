#include <realsense_test_publisher.h>
#include <ros/ros.h>
#include <string>
#include <vector>

float PUBLISH_FREQUENCY = 1.0 / 5.0; // seconds per frame
float PROCESS_FREQUENCY = 1.0; // seconds between process calls

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_realsense_datasets_publisher");
    ros::NodeHandle n;
    // We need two threads to publish and process at the same time
    ros::MultiThreadedSpinner spinner(/*thread_count*/ 2);

    RealsenseTestPublisher test_publisher = RealsenseTestPublisher(&n);

    ros::Timer timer_publish_pointcloud
        = n.createTimer(ros::Duration(PUBLISH_FREQUENCY),
            &RealsenseTestPublisher::publishTestCloud, &test_publisher);

    spinner.spin();
    ros::shutdown();
    return 0;
}
