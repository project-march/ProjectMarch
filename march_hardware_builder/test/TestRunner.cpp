// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  return res;
}
