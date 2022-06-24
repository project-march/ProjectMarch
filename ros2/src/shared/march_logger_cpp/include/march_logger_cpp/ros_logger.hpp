// ROS2 implementation of the logger.
// Date: 24-06-2022
// Created by: George Vegelien, M7.


#ifndef MARCH_LOGGER_CPP_ROS_LOGGER_HPP
#define MARCH_LOGGER_CPP_ROS_LOGGER_HPP

#include "base_logger.hpp"
#include <rclcpp/logger.hpp>

namespace march_logger {
class RosLogger : public BaseLogger {
public:
    RosLogger(const rclcpp::Logger& rclcpp_logger);

    void debug(const std::string& msg) override;
    void info(const std::string& msg) override;
    void warn(const std::string& msg) override;
    void error(const std::string& msg) override;
    void fatal(const std::string& msg) override;

private:
    const rclcpp::Logger rcl_logger_;
};
}  // namespace march_logger



#endif // MARCH_LOGGER_CPP_ROS_LOGGER_HPP
