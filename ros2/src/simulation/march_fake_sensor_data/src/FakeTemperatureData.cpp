// Copyright 2020 Project March.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
// #include "sensor_msgs/msg/temperature.hpp"
// #include <boost/algorithm/string.hpp>
// #include <boost/algorithm/string/split.hpp>
#include "march_fake_sensor_data/FakeTemperatureData.hpp"
#include "march_fake_sensor_data/UniformDistribution.hpp"
#include <deque>
#include <vector>
#include <stdexcept>
#include <string>

/**
 * @file FakeTemperatureData.cpp
 * @brief Responsible for publishing random temperatures for the fake
 * temperature sensors.
 * @details Apply an autoregression filter to the latest 7 randomly generated
 * temperatures to make the temperature less jittery.
 */

/**
 * @param node_name The name that this node will take in ROS.
 * @param autoregression_weights A list of weights that will determine which previous
 * temperature values weigh the most in the calculation of new temperatures.
 */
FakeTemperatureDataNode::FakeTemperatureDataNode(const std::string& node_name, const std::vector<float>&& autoregression_weights):
    Node(node_name),
    latest_temperatures { std::deque<int>(autoregression_weights.size()) },
    autoregression_weights { std::move(autoregression_weights) },
    distribution {0, 0}
{
    // Each value in autoregression_values refers to the weight of
    // the temperatures in latest_temperatures. Therefore, their lengths should
    // match.
    assert(latest_temperatures.size() == autoregression_weights.size());
}

/**
 * @brief Calculate the weighted average of the last random temperatures
 * to avoid jitter.
 * @return A temperature based on the weighted average.
 */
double FakeTemperatureDataNode::calculate_autoregression_temperature() const
{
    double result {0};
    int index {0};
    for (auto temperature : latest_temperatures) {
        result += temperature * autoregression_weights.at(index);
        index += 1;
    }
  return result;
}

/**
 * @brief The range in which new random temperature will be generated.
 * @param minimum_temperature The minimum value (inclusive)
 * @param maximum_temperature The maximum value (inclusive)
 */
void FakeTemperatureDataNode::set_range(int minimum_temperature, int maximum_temperature)
{
    distribution.set_range(minimum_temperature, maximum_temperature);
}

void FakeTemperatureDataNode::add_temperature_publisher(const std::string& sensor_name)
{
    if(sensor_name.empty()) {
        throw std::invalid_argument("Sensor name cannot be empty");
    }
    std::string topic_name = std::string("/march/temperature/") + sensor_name;
    // This is not an important topic, therefore it has lenient QoS settings:
    // - keep only the last sample
    // - attempt to deliver samples, but may lose them if the network is not robust
    //   (best effort)
    // - no attempt is made to persist samples (volatile)
    auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto publisher = this->create_publisher<sensor_msgs::msg::Temperature>(topic_name, qos);
    temperature_publishers.push_back(publisher);
}

void FakeTemperatureDataNode::publish_temperature()
{
    distribution.get_random_number();
    latest_temperatures
//  int random_temperature = randBetween(min_temperature, max_temperature);
//
//  // Update the vector with the latest temperatures by removing the first entry
//  // and adding a new one.
//  latest_temperatures.erase(latest_temperatures.begin());
//  latest_temperatures.push_back(random_temperature);
//
//  double current_temperature = calculateArTemperature(latest_temperatures, ar_values);
//  sensor_msgs::Temperature msg;
//  msg.temperature = current_temperature;
//  msg.header.stamp = ros::Time::now();
//  temperature_pub.publish(msg);
}

///**
// * This callback is called when parameters from the config file are changed
// * during run-time. This method updates the local values which depend on these
// * parameters to make ensure the values are not out-of-date.
// * @param config the config file with all the parameters
// * @param level A bitmask
// */
//void temperatureConfigCallback(march_fake_sensor_data::TemperaturesConfig& config, uint32_t /* level */)
//{
//  // Make sure there is always a possible interval between min and max
//  // temperature.
//  if (config.min_temperature >= config.max_temperature)
//  {
//    config.max_temperature = config.min_temperature + 1;
//  }
//  min_temperature = config.min_temperature;
//  max_temperature = config.max_temperature;
//}
//
//
///**
// * Publish a random temperature within the boundaries of the min and max
// * parameters
// * @param temperature_pub publish the temperature message with this publisher
// */
//void publishTemperature(const ros::Publisher& temperature_pub)
//{
//  int random_temperature = randBetween(min_temperature, max_temperature);
//
//  // Update the vector with the latest temperatures by removing the first entry
//  // and adding a new one.
//  latest_temperatures.erase(latest_temperatures.begin());
//  latest_temperatures.push_back(random_temperature);
//
//  double current_temperature = calculateArTemperature(latest_temperatures, ar_values);
//  sensor_msgs::Temperature msg;
//  msg.temperature = current_temperature;
//  msg.header.stamp = ros::Time::now();
//  temperature_pub.publish(msg);
//}
//

int main(int argc, char** argv)
{
      /*
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
*/
  return 0;
}
