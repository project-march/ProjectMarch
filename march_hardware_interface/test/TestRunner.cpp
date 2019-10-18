// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include <march_hardware_interface/march_hardware.h>
/**
 * Empty class to create coverage report
 */
class TestDummy : public ::testing::Test
{
};

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  auto res = RUN_ALL_TESTS();
  return res;
}
