// Copyright 2020 Project MARCH
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "march_fake_sensor_data/UniformDistribution.hpp"
#include <deque>
#include <vector>
#include <string>

class FakeTemperatureDataNode final : public rclcpp::Node {
    using MessageType = sensor_msgs::msg::Temperature;
    private:
        // Keeps a history of the 7 most recent generated temperatures so it becomes
        // possible to take the weighted average before publishing. This makes the
        // data less jittery, but it is a tradeoff against randomness.
        std::deque<int> latest_temperatures;

        // The weights for the autoregression.
        std::vector<float> autoregression_weights;

        // All the publishers that need to know a temperature. Every iteration, the
        // temperature will be published to these publishers
        std::vector<std::shared_ptr<rclcpp::Publisher<MessageType>>> temperature_publishers;

        // The distribution and the associated generator that will be used to create
        // the random temperatures.
        UniformDistribution distribution;

        // Calculate the weighted average based on the latest temperatures to
        // reduce the jitter in the produced random temperatures.
        double calculate_autoregression_temperature() const;

    public:
        // Constructor that moves the predefined auto regression values into itself.
        FakeTemperatureDataNode(
                const std::string& node_name,
                const std::vector<float>&& autoregression_weights
        );

        void add_temperature_publisher(const std::string& sensor_name);

        void generate_new_temperature();

        // Publish all the fake temperature data to the appropriate topics.
        void publish_temperatures();

        // The temperature should be dynamically adjustable.
        void set_range(int minimum_temperature, int maximum_temperature);
};
