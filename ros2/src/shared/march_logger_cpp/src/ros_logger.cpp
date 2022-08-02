// ROS2 implementation of the logger.
// Date: 24-06-2022
// Created by: George Vegelien, M7.

#include "../include/march_logger_cpp/ros_logger.hpp"
#include <rclcpp/logging.hpp>
namespace march_logger {
RosLogger::RosLogger(const std::shared_ptr<rclcpp::Logger>& rclcpp_logger)
    : rcl_logger_(rclcpp_logger)
{
}

/**
 * \brief Log a message with severity DEBUG.
 * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
 */
void RosLogger::debug(const std::string& msg) const
{
    RCLCPP_DEBUG((*rcl_logger_), msg);
}

/**
 * \brief Log a message with severity INFO.
 * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
 */
void RosLogger::info(const std::string& msg) const
{
    RCLCPP_INFO((*rcl_logger_), msg);
}

/**
 * \brief Log a message with severity WARN.
 * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
 */
void RosLogger::warn(const std::string& msg) const
{
    RCLCPP_WARN((*rcl_logger_), msg);
}

/**
 * \brief Log a message with severity ERROR.
 * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
 */
void RosLogger::error(const std::string& msg) const
{
    RCLCPP_ERROR((*rcl_logger_), msg);
}

/**
 * \brief Log a message with severity FATAL.
 * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
 */
void RosLogger::fatal(const std::string& msg) const
{
    RCLCPP_FATAL((*rcl_logger_), msg);
}

} // namespace march_logger