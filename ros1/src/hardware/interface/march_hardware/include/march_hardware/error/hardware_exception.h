// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#define MARCH_HARDWARE_HARDWARE_EXCEPTION_H
#include "error_type.h"

#include <exception>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace march {
namespace error {
    class HardwareException : public std::exception {
        const ErrorType type_ = ErrorType::UNKNOWN;
        std::runtime_error m;

    public:
        explicit HardwareException(ErrorType type)
            : HardwareException(type, "")
        {
        }

        HardwareException(ErrorType type, const std::string& message)
            : type_(type)
            , m(this->createDescription(message))
        {
        }

        template <typename... Args>
        HardwareException(
            ErrorType type, const std::string& format, Args... args)
            : type_(type)
            , m(this->createDescription(format, args...))
        {
        }

        ~HardwareException() noexcept override = default;

        const char* what() const noexcept override
        {
            return m.what();
        }

        ErrorType type() const noexcept
        {
            return this->type_;
        }

        friend std::ostream& operator<<(
            std::ostream& s, const HardwareException& e)
        {
            s << e.what();
            return s;
        }

    protected:
        std::string createDescription(const std::string& message)
        {
            std::stringstream ss;
            ss << this->type_;
            if (!message.empty()) {
                ss << std::endl;
                ss << message;
            }
            return ss.str();
        }
        template <typename... Args>
        std::string createDescription(const std::string& format, Args... args)
        {
            const size_t size = std::snprintf(
                /*__s=*/nullptr, /*__maxlen=*/0, format.c_str(), args...);
            std::vector<char> buffer(size + 1); // note +1 for null terminator
            std::snprintf(&buffer[0], buffer.size(), format.c_str(), args...);

            return this->createDescription(std::string(buffer.data(), size));
        }
    };

    class NotImplemented : public std::logic_error {
    public:
        explicit NotImplemented(const std::string& function_name)
            : std::logic_error(
                "Function " + function_name + " is not implemented") {};

        NotImplemented(
            const std::string& function_name, const std::string& context)
            : std::logic_error(std::string(/*s=*/"Function ")
                                   .append(function_name)
                                   .append(/*s=*/" is not implemented for ")
                                   .append(context)) {};
    };

} // namespace error
} // namespace march

#endif // MARCH_HARDWARE_HARDWARE_EXCEPTION_H
