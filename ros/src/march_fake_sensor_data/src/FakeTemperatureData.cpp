// Copyright 2018 Project March.

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <dynamic_reconfigure/server.h>
#include <random>

#include <march_fake_sensor_data/TemperaturesConfig.h>

/**
 * @file FakeTemperatureData.cpp
 * @brief Responsible for publishing random temperatures for the fake
 * temperature sensors.
 * @details Apply an autoregression filter to the latest 7 randomly generated
 * temperatures to make the temperature less jittery.
 */

int min_temperature;
int max_temperature;
std::vector<std::string> sensor_names;
std::vector<ros::Publisher> temperature_publishers;

// Store the 7 most recent generated temperatures so we can take the weighted
// average of them when we publish.
std::vector<int> latest_temperatures;

// The weights of the autoregression, the most recent temperature has the
// highest weight.
std::vector<float> ar_values;

// Calculate the weighted average of the latest_temperatures
double calculateArTemperature(std::vector<int> temperatures, std::vector<float> ar_values)
{
  double res = 0;
  for (unsigned int i = 0; i < temperatures.size(); i++)
  {
    res += temperatures.at(i) * ar_values.at(i);
  }
  return res;
}

/**
 * This callback is called when parameters from the config file are changed
 * during run-time. This method updates the local values which depend on these
 * parameters to make ensure the values are not out-of-date.
 * @param config the config file with all the parameters
 * @param level A bitmask
 */
void temperatureConfigCallback(march_fake_sensor_data::TemperaturesConfig& config, uint32_t /* level */)
{
  // Make sure there is always a possible interval between min and max
  // temperature.
  if (config.min_temperature >= config.max_temperature)
  {
    config.max_temperature = config.min_temperature + 1;
  }
  min_temperature = config.min_temperature;
  max_temperature = config.max_temperature;
}

/**
 * @brief Generate number between start and end
 * @param start Lowest number
 * @param end Highest number
 * @return The Random number
 */
int randBetween(int start, int end)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(start, end);
  return dis(gen);
}

/**
 * Publish a random temperature within the boundaries of the min and max
 * parameters
 * @param temperature_pub publish the temperature message with this publisher
 */
void publishTemperature(const ros::Publisher& temperature_pub)
{
  int random_temperature = randBetween(min_temperature, max_temperature);

  // Update the vector with the latest temperatures by removing the first entry
  // and adding a new one.
  latest_temperatures.erase(latest_temperatures.begin());
  latest_temperatures.push_back(random_temperature);

  double current_temperature = calculateArTemperature(latest_temperatures, ar_values);
  sensor_msgs::Temperature msg;
  msg.temperature = current_temperature;
  msg.header.stamp = ros::Time::now();
  temperature_pub.publish(msg);
}

/**
 * @brief Combine the base topic name with sensor name.
 * @param base Leading part of the topic
 * @param name Name of the sensor
 * @return Full topic name
 */
std::string createTopicName(const char* base, const char* name)
{
  char slash[] = "/";
  // The buffer needs more space then the final string length.
  int extra_buffer_size = 100;
  const int kArraySize = static_cast<const int>(strlen(base) + strlen(slash) + strlen(name) + extra_buffer_size);
  // Create a char array of the combined size of all three parts
  char full_topic[kArraySize];
  int error_code = snprintf(full_topic, kArraySize, "%s%s%s", base, slash, name);
  if (error_code < 0 || error_code > kArraySize)
  {
    ROS_ERROR("Error creating topic %s (error code %d)", full_topic, error_code);
  }
  return full_topic;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_fake_sensor_data");
  ros::NodeHandle n;
  ros::Rate rate(10);

  ros::param::get("~min_temperature", min_temperature);
  ros::param::get("~max_temperature", max_temperature);

  int count = 0;
  while (!n.hasParam("/march/joint_names"))
  {
    ros::Duration(0.5).sleep();
    count++;
    if (count > 10)
    {
      ROS_ERROR("Failed to read the joint_names from the parameter server.");
      throw std::runtime_error("Failed to read the joint_names from the parameter server.");
    }
  }

  n.getParam("/march/joint_names", sensor_names);

  // Initialise autoregression variables.
  latest_temperatures = { 0, 0, 0, 0, 0, 0, 0 };
  ar_values = { 0.1, 0.1, 0.1, 0.15, 0.15, 0.2, 0.2 };

  // Create a publisher for each sensor
  for (std::string sensor_name : sensor_names)
  {
    ros::Publisher temperature_pub =
        n.advertise<sensor_msgs::Temperature>(createTopicName("/march/temperature", sensor_name.c_str()), 1000);
    temperature_publishers.push_back(temperature_pub);
  }

  // Make the temperature values dynamic reconfigurable
  dynamic_reconfigure::Server<march_fake_sensor_data::TemperaturesConfig> server;
  dynamic_reconfigure::Server<march_fake_sensor_data::TemperaturesConfig>::CallbackType f;
  server.setCallback(boost::bind(&temperatureConfigCallback, _1, _2));

  while (ros::ok())
  {
    // Loop through all publishers
    for (ros::Publisher temperature_pub : temperature_publishers)
    {
      publishTemperature(temperature_pub);
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
