#include "foot_position_finder.h"
#include <dynamic_reconfigure/server.h>
#include <march_foot_position_finder/parametersConfig.h>
#include <ros/ros.h>

FootPositionFinder* positionFinderLeft;
FootPositionFinder* positionFinderRight;

void parameterCallback(
    march_foot_position_finder::parametersConfig& config, uint32_t level)
{
    positionFinderLeft->readParameters(config, level);
    positionFinderRight->readParameters(config, level);
}

int main(int argc, char** argv)
{
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
