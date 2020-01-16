// Copyright 2019 Project March.
#include <gtest/gtest.h>
#include <gmock/gmock.h>

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}
