#include <march_realsense_reader/pointcloud_parametersConfig.h>
#include <march_realsense_reader/realsense_reader.h>
#include <ros/ros.h>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_realsense_reader");
    ros::NodeHandle n;

    RealSenseReader reader = RealSenseReader(&n);

    dynamic_reconfigure::Server<
        march_realsense_reader::pointcloud_parametersConfig>
        dserver;
    dynamic_reconfigure::Server<
        march_realsense_reader::pointcloud_parametersConfig>::CallbackType f;

    f = std::bind(&RealSenseReader::readConfigCb, &reader,
        std::placeholders::_1, std::placeholders::_2);
    dserver.setCallback(f);

    ros::spin();
    ros::shutdown();
    return 0;
}
