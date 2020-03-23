// Copyright 2020 Project March.
#include "march_hardware/Slave.h"
#include "march_hardware/error/hardware_exception.h"

#include <gtest/gtest.h>

TEST(SlaveTest, CorrectSlaveIndex)
{
  march::Slave slave(1);
  ASSERT_EQ(slave.getSlaveIndex(), 1);
}

TEST(SlaveTest, InvalidSlaveIndex)
{
  ASSERT_THROW(march::Slave slave(0), march::error::HardwareException);
}
