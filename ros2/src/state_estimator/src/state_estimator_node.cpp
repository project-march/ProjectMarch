#include "state_estimator/state_estimator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<StateEstimator>());
    rclcpp::shutdown();
    return 0;
}
