// Copyright 2020 Project March.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
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

/**
 * @brief Add a sensor whose fake temperature will be published on the
 * "/march/temperature/<sensor_name>" topic.
 * @param sensor_name The name of the sensor.
 */
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
    auto publisher = this->create_publisher<MessageType>(topic_name, qos);
    temperature_publishers.push_back(publisher);
}

/**
 * @brief Publish a fake temperature to all the registered temperature publishers.
 */
void FakeTemperatureDataNode::publish_temperatures()
{
    for (auto publisher : temperature_publishers) {
        generate_new_temperature();

        MessageType message;
        message.temperature = calculate_autoregression_temperature();
        message.header.stamp = this->now();
        publisher->publish(message);
    }
}

/**
 * @brief Generate a new pseudo random temperature and add it to the list of latest
 * temperatures.
 */
void FakeTemperatureDataNode::generate_new_temperature()
{
    // "Cycle" through the previous randomly generated temperatures by removing the
    // oldest in the queue and adding a new random number to back of the queue.
    latest_temperatures.pop_front();
    latest_temperatures.push_back(distribution.get_random_number());
}
