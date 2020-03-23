// Copyright 2018 Project March.
#include "mocks/MockAbsoluteEncoder.cpp"
#include "mocks/MockIncrementalEncoder.cpp"

#include <march_hardware/IMotionCube.h>
#include <march_hardware/error/hardware_exception.h>

#include <memory>
#include <utility>

#include <gtest/gtest.h>

class IMotionCubeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    this->mock_absolute_encoder = std::make_unique<MockAbsoluteEncoder>();
    this->mock_incremental_encoder = std::make_unique<MockIncrementalEncoder>();
  }

  std::unique_ptr<MockAbsoluteEncoder> mock_absolute_encoder;
  std::unique_ptr<MockIncrementalEncoder> mock_incremental_encoder;
};

TEST_F(IMotionCubeTest, SlaveIndexOne)
{
  march::IMotionCube imc(1, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::unknown);
  ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeTest, NoActuationMode)
{
  march::IMotionCube imc(1, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::unknown);
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRad)
{
  march::IMotionCube imc(1, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::torque);

  ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
  march::IMotionCube imc(1, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::position);

  ASSERT_THROW(imc.actuateTorque(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, OperationEnabledWithoutActuationMode)
{
  march::IMotionCube imc(1, std::move(this->mock_absolute_encoder), std::move(this->mock_incremental_encoder),
                         march::ActuationMode::unknown);
  ASSERT_THROW(imc.goToOperationEnabled(), march::error::HardwareException);
}
