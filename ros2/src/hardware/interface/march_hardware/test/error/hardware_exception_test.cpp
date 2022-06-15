// Copyright 2020 Project March.
#include <march_hardware/error/hardware_exception.h>

#include <gtest/gtest.h>

using march::error::ErrorType;
using march::error::HardwareException;

TEST(HardwareExceptionTest, TestErrorType)
{
    ErrorType expected = ErrorType::UNKNOWN;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestNoMessage)
{
    ErrorType expected = ErrorType::UNKNOWN;
    HardwareException exception(expected);

    std::stringstream expected_ss;
    expected_ss << expected;

    std::stringstream ss;
    ss << exception;

    ASSERT_EQ(ss.str(), expected_ss.str());
}

TEST(HardwareExceptionTest, TestWhat)
{
    ErrorType expected = ErrorType::UNKNOWN;
    const std::string message = "test";
    HardwareException exception(expected, message);

    std::stringstream expected_ss;
    expected_ss << expected << std::endl << message;

    ASSERT_EQ(expected_ss.str(), std::string(exception.what()));
}

TEST(HardwareExceptionTest, TestMessage)
{
    ErrorType expected = ErrorType::UNKNOWN;
    std::string message = "message";
    HardwareException exception(expected, message);

    std::stringstream expected_ss;
    expected_ss << expected << std::endl << message;

    std::stringstream ss;
    ss << exception;

    ASSERT_EQ(ss.str(), expected_ss.str());
}

TEST(HardwareExceptionTest, TestFormat)
{
    ErrorType expected = ErrorType::UNKNOWN;
    std::string string = "string";
    const int number = -1;
    HardwareException exception(expected, "%s %d", string.c_str(), number);

    std::stringstream expected_ss;
    expected_ss << expected << std::endl << string << " " << number;

    std::stringstream ss;
    ss << exception;

    ASSERT_EQ(ss.str(), expected_ss.str());
}
