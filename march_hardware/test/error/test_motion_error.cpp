// Copyright 2020 Project March.
#include <march_hardware/error/motion_error.h>

#include <gtest/gtest.h>

TEST(TestMotionError, ParseNoMotionError)
{
  ASSERT_EQ(march::error::parseMotionError(0), "");
}

TEST(TestMotionError, ParseCorrectMotionError)
{
  const uint16_t error = 1;
  ASSERT_EQ(march::error::parseMotionError(error), march::error::MOTION_ERRORS[0]);
}

TEST(TestMotionError, ParseMultipleErrors)
{
  const uint16_t error = 0b1000000000001100;
  std::string expected;
  expected += march::error::MOTION_ERRORS[2];
  expected += march::error::MOTION_ERRORS[3];
  expected += march::error::MOTION_ERRORS[15];
  ASSERT_EQ(march::error::parseMotionError(error), expected);
}

TEST(TestDetailedMotionError, ParseNoDetailedMotionError)
{
  ASSERT_EQ(march::error::parseDetailedError(0), "");
}

TEST(TestDetailedMotionError, ParseCorrectDetailedMotionError)
{
  const uint16_t error = 1;
  ASSERT_EQ(march::error::parseDetailedError(error), march::error::DETAILED_MOTION_ERRORS[0]);
}

TEST(TestDetailedMotionError, ParseMultipleDetailedErrors)
{
  const uint16_t error = 0b100001100;
  std::string expected;
  expected += march::error::DETAILED_MOTION_ERRORS[2];
  expected += march::error::DETAILED_MOTION_ERRORS[3];
  expected += march::error::DETAILED_MOTION_ERRORS[8];
  ASSERT_EQ(march::error::parseDetailedError(error), expected);
}
