// Copyright 2020 Project March.

#include "march_fake_sensor_data/FakeTemperatureData.hpp"
#include "march_fake_sensor_data/UniformDistribution.hpp"
#include "march_utility/node_utils.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include <chrono>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

const int DEFAULT_MINIMUM_TEMPERATURE { 0 };
const int DEFAULT_MAXIMUM_TEMPERATURE { 0 };
const std::string MINIMUM_TEMPERATURE_PARAMETER_NAME {
    /*__s=*/"minimum_temperature"
};
const std::string MAXIMUM_TEMPERATURE_PARAMETER_NAME {
    /*__s=*/"maximum_temperature"
};
const std::string LOGGER_NAME { /*__s=*/"fake_temperature" };

/**
 * @file FakeTemperatureData.cpp
 * @brief Responsible for publishing random temperatures for the fake
 * temperature sensors.
 * @details Apply an autoregression filter to the latest 7 randomly generated
 * temperatures to make the temperature less jittery.
 */

/**
 * @param node_name The name that this node will take in ROS.
 * @param autoregression_weights A list of weights that will determine which
 * previous temperature values weigh the most in the calculation of new
 * temperatures.
 */
FakeTemperatureDataNode::FakeTemperatureDataNode(
    const std::string& node_name, std::vector<float>&& autoregression_weights)
    : Node(node_name,
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            /*automatically_declare_parameters_from_overrides=*/true))
    , latest_temperatures { std::deque<int>(autoregression_weights.size()) }
    , autoregression_weights { std::move(autoregression_weights) }
    , minimum_temperature { DEFAULT_MINIMUM_TEMPERATURE }
    , maximum_temperature { DEFAULT_MAXIMUM_TEMPERATURE }
    , distribution { minimum_temperature, maximum_temperature }
{
    this->get_parameter(
        MINIMUM_TEMPERATURE_PARAMETER_NAME, minimum_temperature);
    this->get_parameter(
        MAXIMUM_TEMPERATURE_PARAMETER_NAME, maximum_temperature);
    set_range(minimum_temperature, maximum_temperature);
    // Register a callback that will be called whenever the parameters of this
    // Node are updated. The callback changes the internal state to reflect the
    // new values of the parameters.
    parameter_callback = this->add_on_set_parameters_callback(
        std::bind(&FakeTemperatureDataNode::update_parameters, this,
            std::placeholders::_1));
}

void FakeTemperatureDataNode::initialize()
{
    // Create a temperature publisher for all the different joints.
    for (const auto& sensor : node_utils::get_joint_names(*this)) {
        add_temperature_publisher(sensor);
    }

    // Ensure that new fake temperatures are published every 100 ms.
    timer = this->create_wall_timer(
        100ms, std::bind(&FakeTemperatureDataNode::timer_callback, this));
}

/*
 * @brief Whenever a parameter is changed, the interal state should also change
 * @param parameters Specify the new state of the parameters.
 * @details In ROS 1, this was done with "dynamic_reconfigure". Because ROS 2
 *          has native support for dynamically reconfiguring parameters, this
 *          is done like this.
 */
rcl_interfaces::msg::SetParametersResult
FakeTemperatureDataNode::update_parameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& parameter : parameters) {
        if (parameter.get_name() == MINIMUM_TEMPERATURE_PARAMETER_NAME) {
            minimum_temperature = parameter.get_value<int>();
            set_range(minimum_temperature, maximum_temperature);
        }
        if (parameter.get_name() == MAXIMUM_TEMPERATURE_PARAMETER_NAME) {
            maximum_temperature = parameter.get_value<int>();
            set_range(minimum_temperature, maximum_temperature);
        }
    }
    return result;
}

/**
 * @brief Called whenever the main timer goes off.
 */
void FakeTemperatureDataNode::timer_callback()
{
    publish_temperatures();
}

/**
 * @brief Calculate the weighted average of the last random temperatures
 * to avoid jitter.
 * @return A temperature based on the weighted average.
 */
double FakeTemperatureDataNode::calculate_autoregression_temperature() const
{
    double result { 0 };
    int index { 0 };
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
void FakeTemperatureDataNode::set_range(
    int minimum_temperature, int maximum_temperature)
{
    distribution.set_range(minimum_temperature, maximum_temperature);
}

/**
 * @brief Add a sensor whose fake temperature will be published on the
 * "/march/temperature/<sensor_name>" topic.
 * @param sensor_name The name of the sensor.
 */
void FakeTemperatureDataNode::add_temperature_publisher(
    const std::string& sensor_name)
{
    if (sensor_name.empty()) {
        throw std::invalid_argument("Sensor name cannot be empty");
    }
    std::string topic_name
        = std::string(/*__s=*/"/march/temperature/") + sensor_name;
    // This is not an important topic, therefore it has lenient QoS settings:
    // - keep only the last sample
    // - attempt to deliver samples, but may lose them if the network is not
    // robust
    //   (best effort)
    // - no attempt is made to persist samples (volatile)
    auto qos
        = rclcpp::QoS(/*history_depth=*/1).best_effort().durability_volatile();
    auto publisher = this->create_publisher<MessageType>(topic_name, qos);
    temperature_publishers.push_back(publisher);
}

/**
 * @brief Publish a fake temperature to all the registered temperature
 * publishers.
 */
void FakeTemperatureDataNode::publish_temperatures()
{
    for (const auto& publisher : temperature_publishers) {
        generate_new_temperature();

        MessageType message;
        message.temperature = calculate_autoregression_temperature();
        message.header.stamp = this->now();
        publisher->publish(message);
    }
}

/**
 * @brief Generate a new pseudo random temperature and add it to the list of
 * latest temperatures.
 */
void FakeTemperatureDataNode::generate_new_temperature()
{
    // "Cycle" through the previous randomly generated temperatures by removing
    // the oldest in the queue and adding a new random number to back of the
    // queue.
    latest_temperatures.pop_front();
    latest_temperatures.push_back(distribution.get_random_number());
}
