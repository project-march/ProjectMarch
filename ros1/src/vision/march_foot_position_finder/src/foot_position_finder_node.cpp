/**
 * @author Tuhin Das - MARCH 7
 */

#include "foot_position_finder.h"
#include <boost/thread.hpp>
#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <librealsense2/rs.hpp>
#include <march_foot_position_finder/parametersConfig.h>
#include <ros/ros.h>
#include <thread>

FootPositionFinder* positionFinderLeft;
FootPositionFinder* positionFinderRight;
boost::thread* left;
boost::thread* right;

void parameterCallback(
    march_foot_position_finder::parametersConfig& config, uint32_t level)
{
    left = new boost::thread(boost::bind(&FootPositionFinder::readParameters,
        positionFinderLeft, boost::ref(config), level));
    right = new boost::thread(boost::bind(&FootPositionFinder::readParameters,
        positionFinderRight, boost::ref(config), level));
    left->detach();
    right->detach();
}

int main(int argc, char** argv)
{
    try {
        // Perform realsense hardware reset on all devices
        rs2::context ctx;
        for (auto&& dev : ctx.query_devices()) {
            dev.hardware_reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    } catch (const rs2::error& e) {
        std::string error_message = e.what();
        ROS_WARN("Hardware reset of Realsense cameras failed: %s",
            error_message.c_str());
        // Return error from node so it is restarted
        return -1;
    }

    ros::init(argc, argv, "march_foot_position_finder");
    ros::NodeHandle n;
    positionFinderLeft = new FootPositionFinder(&n, "left");
    positionFinderRight = new FootPositionFinder(&n, "right");

    dynamic_reconfigure::Server<march_foot_position_finder::parametersConfig>
        server;
    dynamic_reconfigure::Server<
        march_foot_position_finder::parametersConfig>::CallbackType f;

    f = std::bind(
        parameterCallback, std::placeholders::_1, std::placeholders::_2);
    server.setCallback(f);

    ros::spin();

    delete positionFinderLeft;
    delete positionFinderRight;

    ros::shutdown();

    return 0;
}
