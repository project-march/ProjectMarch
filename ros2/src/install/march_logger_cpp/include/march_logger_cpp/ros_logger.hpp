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
    // NOLINTNEXTLINE(hicpp-explicit-conversions) This is intended behavior.
    RosLogger(const std::shared_ptr<rclcpp::Logger>& rclcpp_logger);

    explicit RosLogger(const std::string& name);

    void debug(const std::string& msg) const override;
    void info(const std::string& msg) const override;
    void warn(const std::string& msg) const override;
    void error(const std::string& msg) const override;
    void fatal(const std::string& msg) const override;
    BaseLogger* get_logger_append_suffix(const std::string& suffix) const override;

private:
    const std::shared_ptr<rclcpp::Logger> rcl_logger_;
};
} // namespace march_logger

#endif // MARCH_LOGGER_CPP_ROS_LOGGER_HPP
