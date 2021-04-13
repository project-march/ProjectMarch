#include "march_fake_sensor_data/FakeTemperatureData.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a FakeTemperatureNode instance.
    std::vector<float> autoregression_weights { 0.1, 0.1, 0.1, 0.15, 0.15, 0.2,
        0.2 };
    auto temperature = std::make_shared<FakeTemperatureDataNode>(
        "march_fake_temperature_data", std::move(autoregression_weights));
    // Start the timer and create the publishers
    (*temperature).initialize();

    // Execute the temperature node with a single-threaded executor.
    rclcpp::spin(temperature);
    rclcpp::shutdown();
    return 0;
}
