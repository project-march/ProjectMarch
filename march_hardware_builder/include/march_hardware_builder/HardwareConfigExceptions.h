#include <exception>

#include <ros/ros.h>

class HardwareConfigException : public std::exception {

public:
    std::string msg;

    HardwareConfigException() : std::exception(), msg("Invalid hardware configuration")
    {
    }

    explicit HardwareConfigException(std::string msg) : std::exception(), msg(msg)
    {
    }



    const char * what () const throw () {
        ROS_ERROR_STREAM(msg);
        return msg.c_str();
    }
};


class MissingKeyException : public HardwareConfigException {

public:
    std::string keyName;
    std::string objectName;

    explicit MissingKeyException(std::string keyName, std::string objectName) : HardwareConfigException(),
    keyName(keyName), objectName(objectName)
    {
        std::ostringstream stringStream;
        stringStream << "Missing key '" << keyName << "' while creating object '" << objectName <<"'";
        this->msg = stringStream.str();
    }
};

class InvalidValueException : public HardwareConfigException {

public:
    std::string keyName;
    std::string value;
    std::string allowedValues;

    explicit InvalidValueException(std::string keyName, std::string value, std::string allowedValues) : HardwareConfigException(),
    keyName(keyName), value(value), allowedValues(allowedValues)
    {
        std::ostringstream stringStream;
        stringStream << "Key '" << keyName << " can not be of value '" << value << "'. Allowed values: " << allowedValues;
        this->msg = stringStream.str();
    }
};