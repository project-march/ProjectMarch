#ifndef MARCH_UTILITY__LOGGER_COLORS_HPP
#define MARCH_UTILITY__LOGGER_COLORS_HPP

namespace LColor {
/** \brief This constants are here so that one can give colored output in the terminal.
 *
 * \example RCLCPP_INFO(this->get_logger(), "The word %s 'red' %s has a blue color.", LColor::BLUE, LColor::END);
 */
static const char* HEADER = "\033[95m";
static const char* BLUE = "\033[94m";
static const char* CYAN = "\033[96m";
static const char* GREEN = "\033[92m";
static const char* WARNING = "\033[93m";
static const char* FAIL = "\033[91m";
static const char* END = "\033[0m";
static const char* BOLD = "\033[1m";
static const char* UNDERLINE = "\033[4m";
} // namespace LColor

#endif // MARCH_UTILITY__LOGGER_COLORS_HPP
