// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include "mocks/MockTemperatureGES.cpp"

using ::testing::AtLeast;
using ::testing::Return;

TEST(ExampleTest, MockTest)
{
  ASSERT_EQ(3, 3);
  MockTemperatureGES mockTemperatureSensor;
  EXPECT_CALL(mockTemperatureSensor, getTemperature()).Times(AtLeast(1));
  ON_CALL(mockTemperatureSensor, getTemperature()).WillByDefault(Return(10));
  ASSERT_EQ(10, mockTemperatureSensor.getTemperature());
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::FLAGS_gtest_death_test_style = "threadsafe";

  return RUN_ALL_TESTS();
}
