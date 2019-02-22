// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include "march_hardware/TemperatureGES.h"
#include "mocks/MockTemperatureGES.cpp"

using ::testing::Return;
using ::testing::AtLeast;

class ExampleTest : public ::testing::Test
{
protected:
};

TEST_F(ExampleTest, AlwaysTrue)
{
  ASSERT_EQ(3, 3);
}

TEST_F(ExampleTest, MockTest)
{
  MockTemperatureGES mockTemperatureSensor;
  EXPECT_CALL(mockTemperatureSensor, getTemperature())
      .Times(AtLeast(1));
  ON_CALL(mockTemperatureSensor, getTemperature()).WillByDefault(Return(10));
  ASSERT_EQ(10, mockTemperatureSensor.getTemperature());
}

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "march4cpp_test");
  testing::InitGoogleTest(&argc, argv);

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
