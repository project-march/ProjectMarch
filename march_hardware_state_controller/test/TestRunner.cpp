// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include <march_hardware_state_controller/march_hardware_state_controller.h>

/**
 * Empty class to create coverage report
 */
class TestDummy : public ::testing::Test
{
};

TEST_F(TestDummy , AlwaysTrue)
{
    march_hardware_state_controller::MarchHardwareStateController marchController;
    ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  auto res = RUN_ALL_TESTS();
  return res;
}
