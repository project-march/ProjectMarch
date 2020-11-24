#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <random>

class FakeTemperatureDataNode final : public rclcpp::Node {
    private:
        // The range of minimum and maximum temperatures that this node will generate
        // fake temperatures in.
        int minimum_temperature;
        int maximum_temperature;

        // Keeps a history of the 7 most recent generated temperatures so it becomes
        // possible to take the weighted average before publishing. This makes the
        // data less jittery, but it is a tradeoff against randomness.
        std::vector<int> latest_temperatures;

        // The weights for the autoregression.
        std::vector<float> autoregression_values;

        // All the publishers that need to know a temperature. Every iteration, the
        // temperature will be published to these publishers
        std::vector<rclcpp::Publisher<sensor_msgs::msg::Temperature>> temperature_publishers;

        // Calculate the weighted average based on the latest temperatures to
        // reduce the jitter in the produced random temperatures.
        double calculate_autoregression_temperature() const;

    public:
        // Constructor that moves the predefined auto regression values into itself.
        explicit FakeTemperatureDataNode(const std::vector<float>&& autoregression_values);

        void add_temperature_publisher(const std::string&& sensor_name);

        // Publish all the fake temperature data to the appropriate topics.
        void publish_temperatures();

        // The temperature should be dynamically adjustable.
        void set_minimum_temperature(int new_temperature);
        void set_maximum_temperature(int new_temperature);
};
