// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#define MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#include <sstream>
#include <string>

#include <march_hardware/error/error_type.h>
#include <march_hardware/error/hardware_exception.h>

class MissingKeyException : public march::error::HardwareException {
public:
    MissingKeyException(const std::string& key_name, const std::string& object_name)
        : march::error::HardwareException(
            march::error::ErrorType::MISSING_REQUIRED_KEY, createDescription(key_name, object_name))
        , key_name_(key_name)
        , object_name_(object_name)
    {
    }

    /** @brief Override comparison operator */
    friend bool operator==(const MissingKeyException& lhs, const MissingKeyException& rhs)
    {
        return lhs.key_name_ == rhs.key_name_ && lhs.object_name_ == rhs.object_name_;
    }

    /** @brief Override non-equality operator */
    friend bool operator!=(const MissingKeyException& lhs, const MissingKeyException& rhs)
    {
        return !(lhs == rhs);
    }

private:
    std::string key_name_;
    std::string object_name_;

    static std::string createDescription(const std::string& key_name, const std::string& object_name)
    {
        std::ostringstream ss;
        ss << "Missing required key '" << key_name << "' while creating object '" << object_name << "'";
        return ss.str();
    }
};
#endif // MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
