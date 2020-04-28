// Copyright 2020 Project March.
#include "mocks/MockPdoInterface.h"
#include "mocks/MockSdoInterface.h"
#include "march_hardware/EtherCAT/slave.h"
#include "march_hardware/error/hardware_exception.h"

#include <gtest/gtest.h>

class SlaveTest : public testing::Test
{
protected:
  MockPdoInterface mock_pdo;
  MockSdoInterface mock_sdo;
};

TEST_F(SlaveTest, CorrectSlaveIndex)
{
  march::Slave slave(1, this->mock_pdo, this->mock_sdo);
  ASSERT_EQ(slave.getSlaveIndex(), 1);
}

TEST_F(SlaveTest, InvalidSlaveIndex)
{
  ASSERT_THROW(march::Slave(0, this->mock_pdo, this->mock_sdo), march::error::HardwareException);
}
