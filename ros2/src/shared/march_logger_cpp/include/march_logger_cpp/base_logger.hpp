// General interface for the logger class to not get package/framework/company locked.
// Date: 24-06-2022
// Created by: George Vegelien, M7.

#ifndef MARCH_LOGGER_CPP_BASE_LOGGER_HPP
#define MARCH_LOGGER_CPP_BASE_LOGGER_HPP

#include <memory>
#include <vector>

namespace march_logger {
class BaseLogger {

public:
    ~BaseLogger() = default;

    /**
     * \brief Log a message with severity DEBUG.
     * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
     */
    virtual void debug(const std::string& msg) const = 0;

    /**
     * \brief Log a message with severity INFO.
     * \param msg The msg that should be logged.
     */
    virtual void info(const std::string& msg) const = 0;

    /**
     * \brief Log a message with severity WARN.
     * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
     */
    virtual void warn(const std::string& msg) const = 0;

    /**
     * \brief Log a message with severity WARN.
     * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
     */
    virtual void error(const std::string& msg) const = 0;

    /**
     * \brief Log a message with severity FATAL.
     * \param msg The msg that should be logged. (Can be constructed by calling the fstring method)
     */
    virtual void fatal(const std::string& msg) const = 0;

    /**
     * \brief Generates an f string based on random length of arguments.
     * \author iFreilicht, https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
     * \copyright CC0 1.0
     * \example fstring("March %i is %s, 7, "cool") => "March 7 is cool".
     * \todo Change this so that it is a template around every log level so this does not neet to be called separately.
     * @tparam Args Generic template can be anything, should line up with f-string flags.
     * @param format The base string containing f-string flags.
     * @param args A random length of values replacing the f-string flags.
     * (See https://cplusplus.com/reference/cstdio/printf/)
     * @return The formatted string.
     */
    template <typename... Args> static std::string fstring(const std::string& format, Args... args)
    {
        // Extra space for '\0' the null terminator.
        const size_t size = std::snprintf(/*__s=*/nullptr, /*__maxlen=*/0, format.c_str(), args...) + 1;
        if (size <= 0) {
            throw std::runtime_error("Error during formatting.");
        }
        std::vector<char> buffer(size);
        std::snprintf(&buffer[0], size, format.c_str(), args...);
        return { buffer.data(), size - 1 }; // We don't want the '\0' inside
    }
};

} // namespace march_logger

#endif // MARCH_LOGGER_CPP_BASE_LOGGER_HPP
