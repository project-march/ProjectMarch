#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
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
        std::vector<rclcpp::Publisher> temperature_publishers;

        // Calculate the weighted average based on the latest temperatures to
        // reduce the jitter in the produced random temperatures.
        double calculate_autoregression_temperature() const;

    public:
        // Constructor that moves the predefined auto regression values into itself.
        FakeTemperatureDataNode(const std::vector<float>&& autoregression_values) explicit;

        void add_temperature_publisher(const std::string&& sensor_name) explicit;

        // Publish the fake temperature data to the appropriate topic.
        double publish_temperature(const& rclcpp::Publisher publisher);

        // The temperature should be dynamically adjustable.
        void set_minimum_temperature(int new_temperature);
        void set_maximum_temperature(int new_temperature);
};

class UniformDistribution {
    private:
        // Variables that are required for random number generation.
        std::random_device random_device;
        std::mt19937 generator;
        std::uniform_int_distribution<int> distribution;

    public:
        UniformDistribution(const int start, const int end);

        // Assume the start to be 0 if only the end is specified.
        UniformDistribution(const int end) explicit;

        // While the constructor does also create the random device and the random
        // generator, there should also be a possibility to change the range of the
        // generator without replacing it fully. In order to ensure that the pseudo 
        // random numbers have random characteristics, it is necessary that the
        // random device and generator are kept the same.
        void set_range(const int start, const int end);

        // Get a uniformly distributed pseudo random number in the specified range.
        int get_random_number() const;
};
