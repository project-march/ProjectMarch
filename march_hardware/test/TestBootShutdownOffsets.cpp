// Copyright 2018 Project March.

#include "march_hardware/BootShutdownOffsets.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <sstream>

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Return;

class TestBootShutdownOffsets : public ::testing::Test
{
protected:
  const int masterOkByteOffset = 4;
  const int shutdownByteOffset = 3;
  const int shutdownAllowedByteOffset = 2;
};

TEST_F(TestBootShutdownOffsets, ValidValues)
{
  BootShutdownOffsets bootShutdownOffsets(masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);

  EXPECT_EQ(masterOkByteOffset, bootShutdownOffsets.getMasterOkByteOffset());
  EXPECT_EQ(shutdownByteOffset, bootShutdownOffsets.getShutdownByteOffset());
  EXPECT_EQ(shutdownAllowedByteOffset, bootShutdownOffsets.getShutdownAllowedByteOffset());
}
TEST_F(TestBootShutdownOffsets, EmptyConstructor)
{
  BootShutdownOffsets bootShutdownOffsets;

  EXPECT_THROW(bootShutdownOffsets.getMasterOkByteOffset(), std::runtime_error);
  EXPECT_THROW(bootShutdownOffsets.getShutdownByteOffset(), std::runtime_error);
  EXPECT_THROW(bootShutdownOffsets.getShutdownAllowedByteOffset(), std::runtime_error);
}

TEST_F(TestBootShutdownOffsets, InValidValues)
{
  EXPECT_THROW(BootShutdownOffsets bootShutdownOffsets(masterOkByteOffset, -7, shutdownAllowedByteOffset),
               std::runtime_error);
  EXPECT_THROW(BootShutdownOffsets bootShutdownOffsets(-1, shutdownByteOffset, shutdownAllowedByteOffset),
               std::runtime_error);
  EXPECT_THROW(BootShutdownOffsets bootShutdownOffsets(masterOkByteOffset, shutdownByteOffset, -12),
               std::runtime_error);
}

TEST_F(TestBootShutdownOffsets, Equals)
{
  BootShutdownOffsets bootShutdownOffsets1(masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);
  BootShutdownOffsets bootShutdownOffsets2(masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);

  EXPECT_TRUE(bootShutdownOffsets1 == bootShutdownOffsets2);
}
TEST_F(TestBootShutdownOffsets, NotEquals)
{
  BootShutdownOffsets bootShutdownOffsets1(masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);
  BootShutdownOffsets bootShutdownOffsets2(masterOkByteOffset, 17, shutdownAllowedByteOffset);
  BootShutdownOffsets bootShutdownOffsets3(masterOkByteOffset, shutdownByteOffset, 32);

  EXPECT_FALSE(bootShutdownOffsets1 == bootShutdownOffsets2);
  EXPECT_FALSE(bootShutdownOffsets1 == bootShutdownOffsets3);
  EXPECT_FALSE(bootShutdownOffsets2 == bootShutdownOffsets3);
}

TEST_F(TestBootShutdownOffsets, TestStream)
{
  BootShutdownOffsets bootShutdownOffsets(masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);
  std::stringstream ss;
  ss << "bootShutdownOffsets: " << bootShutdownOffsets;
  EXPECT_EQ("bootShutdownOffsets: BootShutdownOffset(masterOk: 4, shutdown: 3, shutdownAllowed: 2)", ss.str());
}