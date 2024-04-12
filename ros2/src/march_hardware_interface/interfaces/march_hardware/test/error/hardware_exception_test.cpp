#ifndef __clang_analyzer__
// NOLINTBEGIN
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

TEST(HardwareExceptionTest, TestErrorType1)
{
    ErrorType expected = ErrorType::INVALID_ACTUATION_MODE;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType2)
{
    ErrorType expected = ErrorType::INVALID_ACTUATE_POSITION;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType3)
{
    ErrorType expected = ErrorType::ENCODER_RESET;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType4)
{
    ErrorType expected = ErrorType::OUTSIDE_HARD_LIMITS;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType5)
{
    ErrorType expected = ErrorType::TARGET_EXCEEDS_MAX_DIFFERENCE;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType6)
{
    ErrorType expected = ErrorType::TARGET_TORQUE_EXCEEDS_MAX_TORQUE;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType7)
{
    ErrorType expected = ErrorType::PDO_OBJECT_NOT_DEFINED;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType8)
{
    ErrorType expected = ErrorType::PDO_REGISTER_OVERFLOW;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType9)
{
    ErrorType expected = ErrorType::WRITING_INITIAL_SETTINGS_FAILED;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType10)
{
    ErrorType expected = ErrorType::NO_SOCKET_CONNECTION;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType11)
{
    ErrorType expected = ErrorType::NOT_ALL_SLAVES_FOUND;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType12)
{
    ErrorType expected = ErrorType::FAILED_TO_REACH_OPERATIONAL_STATE;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType13)
{
    ErrorType expected = ErrorType::INVALID_SLAVE_CONFIGURATION;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType14)
{
    ErrorType expected = ErrorType::NOT_ALLOWED_TO_ACTUATE;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType15)
{
    ErrorType expected = ErrorType::MISSING_URDF_JOINT;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType16)
{
    ErrorType expected = ErrorType::INIT_URDF_FAILED;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType17)
{
    ErrorType expected = ErrorType::INVALID_SW_STRING;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType18)
{
    ErrorType expected = ErrorType::SLAVE_LOST_TIMOUT;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType19)
{
    ErrorType expected = ErrorType::ODRIVE_WRONG_AXIS_NUMBER;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType20)
{
    ErrorType expected = ErrorType::PREPARE_ACTUATION_ERROR;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType21)
{
    ErrorType expected = ErrorType::INVALID_ENCODER_DIRECTION;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType22)
{
    ErrorType expected = ErrorType::BUSY_WAITING_FUNCTION_MAXIMUM_TRIES_REACHED;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType23)
{
    ErrorType expected = ErrorType::INITIAL_TORQUE_NOT_ZERO;
    HardwareException exception(expected);

    ASSERT_EQ(exception.type(), expected);
}

TEST(HardwareExceptionTest, TestErrorType24)
{
    ErrorType expected = ErrorType::MAX_TORQUE_EXCEEDED;
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
// NOLINTEND
#endif
