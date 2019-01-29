// Copyright 2018 Project March.

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include <dynamic_reconfigure/server.h>

#include <march_shared_resources/TopicNames.h>

#include <march_fake_sensor_data/TemperaturesConfig.h>

int min_temperature = 0;
int max_temperature = 50;
std::vector<std::string> sensor_names = {"WRONG", "MORE_WRONG"};
std::vector<ros::Publisher> temperature_publishers;

void callback(march_fake_sensor_data::TemperaturesConfig& config, uint32_t level)
{
  min_temperature = config.min_temperature;
  max_temperature = config.max_temperature;
}

/**
 * Publish a random temperature within the boundaries of the min and max parameters
 * @param temperature_pub publish the temperature message with this publisher
 */
void publishTemperature(ros::Publisher temperature_pub)
{
  // Pick a random value between min and max temperature
  double current_temperature = rand() % (max_temperature - min_temperature) + min_temperature;

  sensor_msgs::Temperature msg;
  msg.temperature = current_temperature;
  temperature_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_fake_sensor_data");
  ros::NodeHandle n;
  ros::Rate rate(10);

  n.getParam(n.getNamespace() + "/min_temperature", min_temperature);
  n.getParam(n.getNamespace() + "/max_temperature", max_temperature);
  n.getParam(n.getNamespace() + "/sensors", sensor_names);
  //TODO(Tim) sensor_names is empty???
  ROS_INFO("%i sensor names loaded", sensor_names.size());

  for (std::string sensor_name : sensor_names)
  {
    //TODO(Tim) add '/' in the topic name
    ros::Publisher temperature_pub = n.advertise<sensor_msgs::Temperature>(TopicNames::temperature + sensor_name, 1000);
    temperature_publishers.push_back(temperature_pub);
  }

  ROS_INFO("%i temperature publishers created", temperature_publishers.size());

  // Make the temperature values dynamic reconfigurable
  dynamic_reconfigure::Server<march_fake_sensor_data::TemperaturesConfig> server;
  dynamic_reconfigure::Server<march_fake_sensor_data::TemperaturesConfig>::CallbackType f;
  server.setCallback(boost::bind(&callback, _1, _2));

  while (ros::ok())
  {
    for (ros::Publisher temperature_pub : temperature_publishers)
    {
      publishTemperature(temperature_pub);
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
