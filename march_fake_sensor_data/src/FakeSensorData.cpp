// Copyright 2018 Project March.

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include <dynamic_reconfigure/server.h>

#include <march_shared_resources/TopicNames.h>

#include <march_fake_sensor_data/TemperaturesConfig.h>

int min_temperature = 0;
int max_temperature = 50;

void callback(march_fake_sensor_data::TemperaturesConfig& config, uint32_t level)
{
  min_temperature = config.min_temperature;
  max_temperature = config.max_temperature;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_fake_sensor_data");
  ros::NodeHandle n;

  ros::Publisher temperature_pub = n.advertise<sensor_msgs::Temperature>(TopicNames::temperature, 1000);
  ros::Rate rate(10);
  n.getParam(n.getNamespace() + "/min_temperature", min_temperature);
  n.getParam(n.getNamespace() + "/max_temperature", max_temperature);

  // Make the temperature values dynamic reconfigurable
  dynamic_reconfigure::Server<march_fake_sensor_data::TemperaturesConfig> server;
  dynamic_reconfigure::Server<march_fake_sensor_data::TemperaturesConfig>::CallbackType f;
  server.setCallback(boost::bind(&callback, _1, _2));

  while (ros::ok())
  {
    // Pick a random value between min and max temperature
    double current_temperature = rand() % (max_temperature - min_temperature) + min_temperature;
    sensor_msgs::Temperature msg;
    msg.temperature = current_temperature;
    temperature_pub.publish(msg);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
