// Copyright 2018 Project March.
#include "mocks/MockSlave.h"
#include "march_hardware/TemperatureGES.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using testing::Return;

class TemperatureGESTest : public ::testing::Test
{
protected:
  MockSlave mock_slave;
};

TEST_F(TemperatureGESTest, ByteOffsetOne)
{
  ASSERT_NO_THROW(march::TemperatureGES(this->mock_slave, 1));
}

TEST_F(TemperatureGESTest, ByteOffsetZero)
{
  ASSERT_NO_THROW(march::TemperatureGES(this->mock_slave, 0));
}

TEST_F(TemperatureGESTest, GetTemperature)
{
  const uint8_t expected_offset = 3;
  const float temperature = 1.0;
  EXPECT_CALL(this->mock_slave.mock_pdo, read32(Eq(this->mock_slave.getSlaveIndex()), Eq(expected_offset)))
      .WillOnce(Return(march::bit32{ .f = temperature }));

  const march::TemperatureGES ges(this->mock_slave, expected_offset);

  ASSERT_FLOAT_EQ(ges.getTemperature(), temperature);
}
